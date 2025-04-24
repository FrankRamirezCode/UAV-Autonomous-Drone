from pymavlink import mavutil
import argparse
import board
import sys

import threading
import math
import time

targetHoverAlt = 2          # Hovers drone 4 meters during mission

boot_time = time.time()
homeXNED = None
homeYNED = None
homeZNED = None

MAV_SYS_STATUS_SENSOR_MAP = {
    0: "3D Gyroscope",
    1: "3D Accelerometer",
    2: "3D Magnetometer",
    3: "Absolute Pressure (Barometer)",
    4: "Differential Pressure",
    5: "GPS",
    6: "Optical Flow",
    7: "Vision Position",
    8: "Laser Position",
    9: "External Ground Truth",
    10: "3D Angular Rate Control",
    11: "Attitude Stabilization",
    12: "Yaw Position",
    13: "Z/Altitude Control",
    14: "XY Position Control",
    15: "Motor Outputs / ESC",
    16: "RC Receiver",
    17: "3D Gyro Calibration",
    18: "3D Accel Calibration",
    19: "3D Mag Calibration",
    20: "GPS Configuration",
    21: "Battery",
    22: "AIRSPEED",
    23: "Collision Prevention",
    24: "Distance Sensor",
    25: "Heading",
    26: "Terrain",
    27: "Reverse Thrust",
}

type_mask = (
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VX_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VY_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_VZ_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_IGNORE |
    mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE)

def GetDroneData(self, parameter):
    with self.telemetry_lock:        
        return self.telemetry.get(parameter, 0)
    
class PX4Interface(threading.Thread):
    def __init__(self, port="/dev/ttyAMA0", baud=921600, source_system=2):
        super().__init__(daemon=True)
        self.telemetry_lock = threading.Lock()
        self.running = True
        self.master = None       

        self.telemetry = {
            "battery_voltage": 0.0, "battery_remaining": 0.0,
            "gps_fix_type": 0, "satellites_visible": 0,
            "startingLatitude": None, "startingLongitude": None, "startingAltitude": None,
            "startingNED_x": 0.0, "startingNED_y": 0.0, "startingNED_z": 0.0,
            "startingNED_vx": 0.0, "startingNED_vy": 0.0, "startingNED_vz": 0.0,
            "sensorsPresent": 0, "sensorsEnabled": 0, "sensorsHealth": 0,
            "mode": "UNKNOWN", "armed": None,

            "roll": 0.0, "pitch": 0.0, "yaw": 0.0,
            "landed": True,
        }
        
    def ConnectToDrone(self, port='/dev/ttyAMA0', baud=921600, source_system=2, timeout=5):
        self.master = mavutil.mavlink_connection(port, baud=baud, source_system=source_system)
        if (self.master.wait_heartbeat(timeout=timeout)):
            return True
        else:
            return False                      
    
    def GetBatteryVoltage(self, timeout=3):
        deadline = time.time() + timeout
        while time.time() < deadline:  
            voltage = GetDroneData(self, "battery_voltage")             
            if voltage != 0:
                return voltage, GetDroneData(self, "battery_remaining")
            time.sleep(0.1)
        return False

    def GetHomeGPS(self, timeout=5):        
        deadline = time.time() + timeout
        while time.time() < deadline:
            fixType = GetDroneData(self, "gps_fix_type")
            if fixType >= 3:
                visibleSats = GetDroneData(self, "satellites_visible")
                print(f"[INFO] GPS Lock Obtained - fix_type {fixType}, satellites: {visibleSats}")
                while True:                    
                    latitude, longitude = GetDroneData(self, "Latitude"), GetDroneData(self, "Longitude")
                    if latitude != 0 and longitude != 0:
                        return latitude, longitude, GetDroneData(self, "Altitude")
                    time.sleep(0.1)
            time.sleep(0.1)                
        return False
    
    def GetNEDPosition(self, timeout=2):
        deadline = time.time() + timeout
        while time.time() < deadline:
            xNED, yNED, zNED = GetDroneData(self, "NED_x"), GetDroneData(self, "NED_y"), GetDroneData(self, "NED_z")
            if zNED is not None and abs(zNED) > 0.01:                       # Confirm z is not 0
                return xNED, yNED, zNED
            time.sleep(0.1)                
        return False
    
    def PreFlightChecksOK(self, timeout=10):
        deadline = time.time() + timeout
        while time.time() < deadline:
            present, enabled, health = GetDroneData(self, "sensorsPresent"), GetDroneData(self, "sensorsEnabled"), GetDroneData(self, "sensorsHealth")
            if present != 0 and enabled != 0 and health != 0:                    
                if (present & enabled & health) == enabled:
                    return True
                else:
                    print(f"Unhealthy sensor data obtained")
                    self.PrintUnhealthySensors(present, enabled, health)                     
            time.sleep(1)
    
    def PrintUnhealthySensors(self, present, enabled, health):
        for bit, name in MAV_SYS_STATUS_SENSOR_MAP.items():
            mask = 1 << bit

            sensor_enabled = bool(enabled & mask)
            sensor_present = bool(present & mask)
            sensor_healthy = bool(health & mask)

            if sensor_enabled:
                if not sensor_present:
                    print(f" - {name}: Enabled but NOT present")
                elif not sensor_healthy:
                    print(f" - {name}: Enabled and present, but UNHEALTHY")
                # else:
                #     print(f"   -   {name} ({bit}): All good.")
            else:
                if sensor_present:
                    print(f" - {name}: Present but not enabled")

    def SendDummySetpoints(self, duration_sec=6, hz=10):
        homeNED = self.GetNEDPosition(timeout=10)
        if homeNED:
            homeXNED, homeYNED, homeZNED = homeNED
        print(f"[INFO] Priming with dummy position setpoints for {duration_sec} seconds at {hz} Hz")
        
        target_z = homeZNED - targetHoverAlt

        interval = 1.0 / hz
        count = int(duration_sec * hz)
        for _ in range(count):
            self.master.mav.set_position_target_local_ned_send(
                int((time.time() - boot_time) * 1e3) % (2**32),
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                type_mask,                      # Use Position only
                homeXNED, homeYNED, target_z,   # Use x, y, z Only
                0, 0, 0,                        # velocity ignored
                0, 0, 0,                        # Acceleration ignored
                0, 0)                           # Yaw, yaw rate ignored                            
            time.sleep(interval)     
    
    def SendOffboardRequest(self):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
            6,                                  # OFFBOARD mode in PX4
            0, 0, 0, 0, 0)

    def SendArmRequest(self):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,                                  # Confirmation
            1,                                  # 1 = arm, 0 = disarm
            0, 0, 0, 0, 0, 0)

    def run(self):        
        while self.running:
            with self.telemetry_lock:
                msg = self.master.recv_match(blocking=False)
                if msg:
                    msg_type = msg.get_type()

                    if msg_type == "HEARTBEAT":
                        self.telemetry["mode"] = mavutil.mode_string_v10(msg)
                        self.telemetry["armed"] = bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
                    elif msg_type == "GPS_RAW_INT":
                        gpsFix = msg.fix_type
                        if gpsFix >=3:
                            self.telemetry["gps_fix_type"] = gpsFix
                            self.telemetry["satellites_visible"] = msg.satellites_visible
                    elif msg_type == "GLOBAL_POSITION_INT":
                        self.telemetry["Latitude"] = msg.lat / 1e7  # degrees
                        self.telemetry["Longitude"] = msg.lon / 1e7
                        self.telemetry["Altitude"] = msg.alt / 1000.0  # meters above mean sea level
                    elif msg_type == "LOCAL_POSITION_NED":
                        self.telemetry["NED_x"] = msg.x
                        self.telemetry["NED_y"] = msg.y
                        self.telemetry["NED_z"] = msg.z
                        self.telemetry["NED_vx"] = msg.vx
                        self.telemetry["NED_vy"] = msg.vy
                        self.telemetry["NED_vz"] = msg.vz
                    elif msg_type == "SYS_STATUS":
                        if hasattr(msg, "voltage_battery"):
                            self.telemetry["battery_voltage"] = msg.voltage_battery / 1000.0
                        if hasattr(msg, "battery_remaining"):
                            self.telemetry["battery_remaining"] = msg.battery_remaining
                        if hasattr(msg, "onboard_control_sensors_present"):
                            self.telemetry["sensorsPresent"] = msg.onboard_control_sensors_present
                            self.telemetry["sensorsEnabled"] = msg.onboard_control_sensors_enabled
                            self.telemetry["sensorsHealth"] = msg.onboard_control_sensors_health
                    elif msg_type == "STATUSTEXT":
                        text = msg.text
                        if "prearm" in text.lower() or "arm" in text.lower():
                            print(f"[PX4 WARNING]: {text}")
                        else:
                            print(f"[PX4]: {text}")
        time.sleep(0.01)

    # def send_position_setpoint(self, x, y, z, yaw=0):
    #     self.master.mav.set_position_target_local_ned_send(
    #         int(time.time() * 1e6),  # time_boot_us
    #         self.master.target_system,
    #         self.master.target_component,
    #         mavutil.mavlink.MAV_FRAME_LOCAL_NED,
    #         0b0000111111000111,  # Enable position and yaw only
    #         x, y, z,             # Position (NED)
    #         0, 0, 0,             # Velocity (ignored)
    #         0, 0, 0,             # Acceleration (ignored)
    #         yaw, 0               # Yaw, yaw rate
    #     )



    def disarm(self):
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,                                  # Confirmation
            0,                                  # 1 = arm, 0 = disarm
            0, 0, 0, 0, 0, 0)


    def takeoff(self, target_altitude=2.0):
        print("[INFO] Streaming dummy setpoints...")
        for _ in range(100):  # 5 seconds at 20Hz
            self.send_position_setpoint(0, 0, -0.2)
            time.sleep(0.05)

        self.arm()
        self.set_mode_offboard()

        print(f"[INFO] Taking off to {target_altitude} meters...")
        ned_z = -target_altitude  # NED: up is negative
        for _ in range(200):  # 10 seconds climb
            self.send_position_setpoint(0, 0, ned_z)
            time.sleep(0.05)

        print("[INFO] Hovering at target altitude.")

    def StopThread(self):
        self.running = False
        self.join()  # This replaces checking .is_alive() and joining the listener manually











def Main():
    # Connect to Pixhawk
    # Check Battery Level
    # Get Home GPS Coordinates
    # Get NED Coordinates
    # Check PreFlightHealth
    # 
    should_skip = False


    drone = PX4Interface()    

    while True:
        print("[INFO] Attempting to connect to Pixhawk")
        if drone.ConnectToDrone(timeout=2):
            print(f"[INFO] Successfully connected to Pixhawk on system {drone.master.target_system}") 
            break           
        else:
            print(f"[ERROR] No communication with Pixhawk - Check serial connections and try again!")
            while True:
                choice = input("Would you like to try again now? (y/n): ").strip().lower()
                if choice == 'n':
                    print("[INFO] Exiting script")
                    sys.exit(1)
                elif choice == 'y':
                    break
                elif choice == "skip":
                    should_skip = True
                    break   
                elif choice != 'y' and choice != 'n':
                    print("Please enter (y/n): ")
                    continue  
        if should_skip:
            break         
      
    drone.start()

    while True:
        print("[INFO] Attempting to receive battery voltage")
        voltage, remainingBattery = drone.GetBatteryVoltage(timeout=3)

        if not voltage == False:
            print(f"[INFO] Battery Voltage = {voltage}V  -  {remainingBattery}% remaining")
            break
        else:
            print(f"[WARNING] Cannot read battery voltage!")
            while True:
                choice = input("Would you like to try again now? (y/n): ").strip().lower()
                if choice == 'n':
                    print("[INFO] Exiting script")
                    sys.exit(1)
                elif choice == 'y':
                    break
                elif choice == "skip":
                    should_skip = True
                    break   
                elif choice != 'y' and choice != 'n':
                    print("Please enter (y/n): ")
                    continue  
        if should_skip:
            break

    while True:        
        print("[INFO] Attempting to receive GPS Coordinates")
        homeGPS = drone.GetHomeGPS(timeout=1)
        if homeGPS:
            startlat, startlong, startalt = homeGPS
            print(f"[INFO] Home position: Lat={startlat}, Lon={startlong}, Alt={startalt} m")
            break           
        else:
            if not GetDroneData(drone, "gps_fix_type") >= 3:
                print(f"[ERROR] Coundn't get GPS Fix")
            else:
                print(f"[ERROR] Latitude and Longitude equal to 0")
            while True:
                choice = input("Would you like to try again now? (y/n): ").strip().lower()
                if choice == 'n':
                    print("[INFO] Exiting script")
                    sys.exit(1)
                elif choice == 'y':
                    break
                elif choice == "skip":
                    should_skip = True
                    break   
                elif choice != 'y' and choice != 'n':
                    print("Please enter (y/n): ")
                    continue  
        if should_skip:
            break

    while True:
        print("[INFO] Attempting to receive NED position")
        homeNED = drone.GetNEDPosition(timeout=10)
        if homeNED:
            homeXNED, homeYNED, homeZNED = homeNED
            print(f"[INFO] NED position: NED_x={homeXNED}, NED_y={homeYNED}, NED_z={homeZNED}")
            break           
        else:
            print(f"[ERROR] Couldn't receive accurate NED position")
            while True:
                choice = input("Would you like to try again now? (y/n): ").strip().lower()
                if choice == 'n':
                    print("[INFO] Exiting script")
                    sys.exit(1)
                elif choice == 'y':
                    break
                elif choice == "skip":
                    should_skip = True
                    break   
                elif choice != 'y' and choice != 'n':
                    print("Please enter (y/n): ")
                    continue  
        if should_skip:
            break

    while True:
        print("[INFO] Attempting to receive Pre Arm Health")
        if drone.PreFlightChecksOK(timeout=10):
            print("[INFO] All Pre Arm checks satisfied")
            break           
        else:
            print("Did not pass pre flight checks")
            while True:
                choice = input("Would you like to try again now? (y/n): ").strip().lower()
                if choice == 'n':
                    print("[INFO] Exiting script")
                    sys.exit(1)
                elif choice == 'y':
                    break
                elif choice == "skip":
                    should_skip = True
                    break   
                elif choice != 'y' and choice != 'n':
                    print("Please enter (y/n): ")
                    continue  
        if should_skip:
            break


    try:
        while not drone.master.motors_armed():
            drone.SendDummySetpoints()
            drone.SendOffboardRequest()
            drone.SendArmRequest()

        while GetDroneData(drone, ("NED_z")) > homeZNED - targetHoverAlt:
            drone.master.mav.set_position_target_local_ned_send(
                int((time.time() - boot_time) * 1e3) % (2**32),
                drone.master.target_system,
                drone.master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                type_mask,                      # Use Position only
                homeXNED, homeYNED, GetDroneData(drone, ("NED_z")) - 0.1,   # Use x, y, z Only
                0, 0, 0,                        # velocity ignored
                0, 0, 0,                        # Acceleration ignored
                0, 0)                           # Yaw, yaw rate ignored 
            time.sleep(0.1)

        print(f"[INFO] Hovering at {targetHoverAlt} meters for 1 second")
        time.sleep(1)
        print(f"[INFO] Now landing")

        while GetDroneData(drone, ("NED_z")) < homeZNED:
            drone.master.mav.set_position_target_local_ned_send(
                int((time.time() - boot_time) * 1e3) % (2**32),
                drone.master.target_system,
                drone.master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                type_mask,                      # Use Position only
                homeXNED, homeYNED, GetDroneData(drone, ("NED_z")) + 0.1,   # Use x, y, z Only
                0, 0, 0,                        # velocity ignored
                0, 0, 0,                        # Acceleration ignored
                0, 0)                           # Yaw, yaw rate ignored 
            time.sleep(0.1)


        print(f"[INFO] Mission Complete")

        print("END")

    except KeyboardInterrupt:
        print("\n[INFO] Mission interrupted by user.")
        for _ in range(5):
            drone.disarm()
            time.sleep(0.1)
        sys.exit(0)



if __name__ == "__main__":
    Main()