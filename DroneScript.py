from pymavlink import mavutil
from rpi_ws281x import *
import argparse
import ydlidar
import board
import sys

import threading
import math
import time

targetHoverAlt = 4          # Hovers drone 4 meters during mission

boot_time = time.time()

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

LED_COUNT      = 40     # Number of LED pixels.
LED_PIN        = 18      # GPIO pin connected to the pixels (18 uses PWM!).
LED_FREQ_HZ    = 800000  # LED signal frequency in hertz (usually 800khz)
LED_DMA        = 10      # DMA channel to use for generating a signal (try 10)
LED_BRIGHTNESS = 64      # Set to 0 for darkest and 255 for brightest
LED_INVERT     = False   # True to invert the signal (when using NPN transistor level shift)
LED_CHANNEL    = 0       # set to '1' for GPIOs 13, 19, 41, 45 or 53


strip = Adafruit_NeoPixel(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)
strip.begin()

blink_thread = None

def connect_to_drone():
    master = mavutil.mavlink_connection("/dev/ttyAMA0", baud=921600, source_system=2)
    master.wait_heartbeat()
    print(f"[INFO] Heartbeat received from drone system {master.target_system}")
    return master
    
def wait_for_gps(master):
    print("[INFO] Waiting for GPS lock...")
    deadline = time.time() + 30
    while time.time() < deadline:
        msg = master.recv_match(type='GPS_RAW_INT', blocking=True, timeout=5)
        if msg and msg.fix_type >= 3:
            print(f"[INFO] GPS OK - fix_type {msg.fix_type}, satellites: {msg.satellites_visible}")
            return True   
    print("[ERROR] No GPS Lock")
    return False
    
def get_gps_coordinates(master):
    print("[INFO] Getting Current GPS Coordinates")
    while True:
        msg = master.recv_match(blocking=True, timeout=10)

        if msg:
            if msg.get_type() == "GLOBAL_POSITION_INT":
                print("[INFO] Received GPS data: GLOBAL_POSITION_INT")
                lat = msg.lat / 1e7
                lon = msg.lon / 1e7
                alt = msg.relative_alt / 1000.0
                return lat, lon, alt

            elif msg.get_type() == "GPS_RAW_INT":
                print("[INFO] Received GPS data: GPS_RAW_INT")
                lat = msg.lat / 1e7
                lon = msg.lon / 1e7
                alt = msg.alt / 1000.0
                return lat, lon, alt
        else:
            print("No data received within timeout.")

def wait_for_local_position(master):
    print("[INFO] Waiting for valid LOCAL_POSITION_NED...")
    while True:
        msg = master.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=2)
        if msg:
            print("[INFO] LOCAL_POSITION_NED received.")
            return
        print("[WARN] Still waiting for LOCAL_POSITION_NED...")
        time.sleep(1)

def get_current_local_position(master):
    print("[INFO] Getting current local position...")
    while True:
        msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=2)
        if msg:
            print(f"[INFO] Initial local position: x={msg.x:.2f}, y={msg.y:.2f}, z={msg.z:.2f}")
            return msg.x, msg.y, msg.z

def set_offboard_mode(master):
    print("[INFO] Switching to OFFBOARD mode...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_DO_SET_MODE,
        0,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        6,  # OFFBOARD mode in PX4
        0, 0, 0, 0, 0
    )

def wait_for_offboard(master, timeout=5):
    print("[INFO] Waiting for OFFBOARD mode confirmation...")
    end = time.time() + timeout
    while time.time() < end:
        hb = master.recv_match(type='HEARTBEAT', blocking=False)
        if hb and hb.get_srcSystem() == master.target_system:
            custom_mode = hb.custom_mode
            main_mode = (custom_mode >> 16) & 0xFF  # PX4 encoding
            print(f"[DEBUG] custom_mode: {custom_mode}, main_mode: {main_mode}")
            if main_mode == 6:  # OFFBOARD
                print("[INFO] OFFBOARD mode confirmed (PX4 main_mode == 6).")
                return True
        time.sleep(0.2)
    print("[ERROR] OFFBOARD mode not confirmed.")
    return False

class OffboardThread(threading.Thread):
    def __init__(self, master, x=0, y=0, z=targetHoverAlt):
        super().__init__()
        self.master = master
        self.x = x
        self.y = y
        self.z = z
        self.running = True

    def run(self):
        print("[INFO] OFFBOARD setpoint thread started.")
        while self.running:
            self.master.mav.set_position_target_local_ned_send(
                int((time.time() - boot_time) * 1e3) % (2**32),
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                0b0000111111000111,  # Only position enabled
                self.x,  # Forward/Backward
                self.y,  # Left/Right
                -self.z,  # Up/Down (NED frame)
                0, 0, 0,
                0, 0, 0,
                0, 0
            )
            time.sleep(0.1)

    def stop(self):
        self.running = False
        print("[INFO] OFFBOARD setpoint thread stopped.")

def check_preflight_health(master, timeout=10):
    print("[INFO] Checking preflight health status...")
    deadline = time.time() + timeout

    while time.time() < deadline:
        msg = master.recv_match(type='SYS_STATUS', blocking=True, timeout=1)
        if msg:
            sensors_present = msg.onboard_control_sensors_present
            sensors_enabled = msg.onboard_control_sensors_enabled
            sensors_health  = msg.onboard_control_sensors_health

            if (sensors_present & sensors_enabled & sensors_health) == sensors_enabled:
                print("[INFO] All required sensors report healthy status.")
                return True
            else:
                print("[ERROR] One or more sensors are unhealthy:")
                print_unhealthy_sensors(sensors_present, sensors_enabled, sensors_health)
        else:
            print("[WARNING] Did not receive SYS_STATUS message")

        time.sleep(1)

    print("[ERROR] Preflight health checks failed.")
    return False

def print_unhealthy_sensors(present, enabled, health):
    unhealthy = enabled & ~health
    print(f"\n Decoding sensor health:")
    print(f"Unhealthy bitmask: {bin(unhealthy)}")

    if unhealthy == 0:
        print(" All enabled sensors are healthy.")
        return

    print(" Unhealthy sensors:")
    for i in range(32):
        if unhealthy & (1 << i):
            sensor_name = MAV_SYS_STATUS_SENSOR_MAP.get(i, f"Unknown sensor (bit {i})")
            print(f"  - Bit {i}: {sensor_name}")

class LidarThread(threading.Thread):
    def __init__(self):
        super().__init__()
        self.lock = threading.Lock()
        self.sector_closest = [float('inf')] * 20  # Closest obstacle in each sector
        self.running = True

        ydlidar.os_init()
        self.lidar = ydlidar.CYdLidar()

        ports = ydlidar.lidarPortList()
        port = "/dev/ydlidar"
        for key, value in ports.items():
            port = value

        # Set LIDAR properties
        self.lidar.setlidaropt(ydlidar.LidarPropSerialPort, port)
        self.lidar.setlidaropt(ydlidar.LidarPropSerialBaudrate, 230400)
        self.lidar.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
        self.lidar.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
        self.lidar.setlidaropt(ydlidar.LidarPropScanFrequency, 12.0)
        self.lidar.setlidaropt(ydlidar.LidarPropSampleRate, 9)
        self.lidar.setlidaropt(ydlidar.LidarPropSingleChannel, False)
        self.lidar.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0)
        self.lidar.setlidaropt(ydlidar.LidarPropMinAngle, -180.0)
        self.lidar.setlidaropt(ydlidar.LidarPropMaxRange, 16.0)
        self.lidar.setlidaropt(ydlidar.LidarPropMinRange, 0.01)

    def run(self):
        print("[INFO] Starting LIDAR scanning thread...")
        if not self.lidar.initialize():
            print("[ERROR] Failed to initialize LIDAR.")
            return

        if not self.lidar.turnOn():
            print("[ERROR] Failed to turn on LIDAR.")
            return

        scan = ydlidar.LaserScan()

        while self.running and ydlidar.os_isOk():
            if self.lidar.doProcessSimple(scan):
                # New scan = reset current closest readings
                temp_closest = [float('inf')] * 20

                for point in scan.points:
                    distance = point.range
                    if 0.01 < distance < 16.0:
                        angle = point.angle * (180.0 / math.pi) + 180.0  # -180→180 → 0→360
                        sector = min(int(angle // 18), 19)
                        temp_closest[sector] = min(temp_closest[sector], distance)

                # Overwrite full sector_closest every scan
                with self.lock:
                    self.sector_closest = [
                        d if d != float('inf') else None  # or float('inf')
                        for d in temp_closest
                    ]
            else:
                print("[WARN] Failed to get LIDAR data.")

        self.lidar.turnOff()
        self.lidar.disconnecting()
        print("[INFO] LIDAR scanning stopped.")


    def stop(self):
        self.running = False

    def get_sector_closest(self):
        with self.lock:
            return self.sector_closest[:]

def arm_drone(master):
    print("[INFO] Attempting to arm drone...")
    for attempt in range(5):
        print(f"[INFO] Arming attempt {attempt+1}/5")
        master.mav.command_long_send(
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            0,
            1, 0, 0, 0, 0, 0, 0
        )
        time.sleep(0.5)
        # return True
        # listen_for_statustext(master, timeout=5)

        # Check if motors are armed
        #timeout = time.time() + 3
        # while time.time() < timeout:
        #     if master.motors_armed():
        #         print("[INFO] Drone is armed!")
        #         return True
        #     time.sleep(0.1)
        return True

        print("[WARN] Arming timed out.")

    print("[ERROR] Drone failed to arm after multiple attempts.")
    return False

def listen_for_statustext(master, timeout=5):
    print(f"[DEBUG] Listening for PX4 status messages containing ...")
    end = time.time() + timeout
    while time.time() < end:
        msg = master.recv_match(type='STATUSTEXT', blocking=False)
        if msg in msg.text.lower():
            print(f"[PX4 STATUS] {msg.severity} - {msg.text}")
            return True
        time.sleep(0.1)
    print(f"[WARN] No STATUSTEXT containing received.")
    return False

def takeoff_to_altitude(master, home_z, target_altitude=targetHoverAlt, climb_rate=1.0):
    print(f"[INFO] Taking off to {target_altitude} meters above home altitude...")

    target_z = home_z - target_altitude  # NED frame: lower z = higher up
    type_mask = (
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_X_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Y_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_Z_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AX_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AY_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_AZ_IGNORE |
        mavutil.mavlink.POSITION_TARGET_TYPEMASK_YAW_RATE_IGNORE
    )

    while True:
        # Get current altitude
        msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=1)
        if msg:
            current_z = msg.z
            print(f"[DEBUG] Current Z: {current_z:.2f} | Target Z: {target_z:.2f}")
            if current_z <= target_z:  # reached or exceeded desired height
                print("[INFO] Target altitude reached. Hovering...")
                break

        # Send vertical velocity setpoint (upward)
        timestamp = int((time.time() - boot_time) * 1000) % 2**32
        master.mav.set_position_target_local_ned_send(
            timestamp,
            master.target_system,
            master.target_component,
            mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            type_mask,
            0, 0, 0,         # Position ignored
            0, 0, -climb_rate,  # Z velocity: negative = up
            0, 0, 0,
            0, 0
        )

        time.sleep(0.1)

def land_drone(master):
    print("[INFO] Initiating landing...")
    master.set_mode("LAND")
    print("[INFO] Landing command sent")

def wait_for_landing(master):
    if wait_until_landed(master):
        return True
    return has_landed_by_position(master)

def wait_until_landed(master, timeout=30):
    print("[INFO] Waiting for drone to land...")
    start_time = time.time()

    while time.time() - start_time < timeout:
        # Check EXTENDED_SYS_STATE if available
        msg = master.recv_match(type='EXTENDED_SYS_STATE', blocking=True, timeout=1)
        if msg and msg.landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND:
            print("[INFO] Drone has landed (state).")
            return True

        # Fallback: Check altitude
        pos = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
        if pos:
            alt = pos.relative_alt / 1000.0
            print(f"[INFO] Altitude: {alt:.2f} m")
            if alt < 0.2:
                print("[INFO] Drone has landed (altitude fallback).")
                return True

        time.sleep(1)

    print("[WARN] Landing timeout exceeded.")
    return False

def has_landed_by_position(master, threshold_z=-0.1, threshold_vz=0.1, timeout=30):
    start_time = time.time()

    while time.time() - start_time < timeout:
        msg = master.recv_match(type='LOCAL_POSITION_NED', blocking=True, timeout=2)
        if msg:
            if msg.z > threshold_z and abs(msg.vz) < threshold_vz:
                print(f"[INFO] Landed at z={msg.z:.2f}, vz={msg.vz:.2f}")
                return True
        time.sleep(1)

    print("[WARN] Drone did not land within expected time.")
    return False

def change_led_color(color):
    for i in range(strip.numPixels()):
        strip.setPixelColor(i, color)
    strip.show()

class BlinkThread(threading.Thread):
    def __init__(self, color, rate):
        super().__init__()
        self.color = color
        self.interval = 1.0 / (rate * 2)  # Half period for on/off
        self.running = True

    def run(self):
        while self.running:
            for i in range(LED_COUNT):
                strip.setPixelColor(i, self.color)
            strip.show()
            time.sleep(self.interval)

            if not self.running:  # Check again before turning off
                break

            for i in range(LED_COUNT):
                strip.setPixelColor(i, Color(0, 0, 0))
            strip.show()
            time.sleep(self.interval)

    def stop(self):
        self.running = False

def set_blink(color, rate):
    global blink_thread
    if blink_thread and blink_thread.is_alive():
        blink_thread.stop()
        blink_thread.join()
    blink_thread = BlinkThread(color, rate)
    blink_thread.start()

def stop_blink():
    global blink_thread
    if blink_thread and blink_thread.is_alive():
        blink_thread.stop()
        blink_thread.join()


def blink_until_solid():
    set_blink(Color(0,255,0),2)
    time.sleep(2)
    stop_blink()
    set_blink(Color(0,255,0),5)
    time.sleep(2)
    stop_blink()
    set_blink(Color(0,255,0),25)
    time.sleep(2)
    stop_blink()
    set_blink(Color(0,255,0),40)
    time.sleep(2)
    stop_blink()

    # Final solid ON
    change_led_color(Color(0, 255, 0))

def main():
    try:
        set_blink(Color(255,255,0), 2)      #255,255,0 is YELLOW
        master = connect_to_drone()        
        # if not wait_for_gps(master):
        #     print("No GPS Lock after 30 seconds. Move drone and try again!")
        #     stop_blink()
        #     set_blink(Color(255,0,0), 10)
        #     time.sleep(3)
        #     stop_blink()
        #     return
        
        # latitude, longitude, altitude = get_gps_coordinates(master)
        # if latitude == 0 and longitude == 0 :
        #     print("[ERROR] GPS Data Invalid - Latitude and Longitude = 0!")
        #     stop_blink()
        #     set_blink(Color(255,0,0), 10)
        #     time.sleep(3)
        #     stop_blink()      
        #     return
        
        # print(f"[INFO] Current GPS: Latitude: {latitude}, Longitude: {longitude}, Altitude: {altitude} meters")
        
        # stop_blink()
        #change_led_color(Color(0,255,0))


        wait_for_local_position(master)

        home_x, home_y, home_z = get_current_local_position(master)

        target_z = home_z - targetHoverAlt  # subtract because NED: down is positive

        if check_preflight_health(master):
            print("[INFO] Preflight checklist Passed")
            lidar_thread = LidarThread()
            lidar_thread.start()
            time.sleep(4)
            

            obstacle_detected = True

            while obstacle_detected == True:
                closest = lidar_thread.get_sector_closest()

                obstacle_detected = False  # Reset once at the start of the loop

                for i, dist in enumerate(closest):
                    if dist is not None and dist < 0.4:                 #Minimum distance should be 0.4 to avoid obstacles from propellors.
                        obstacle_detected = True
                        break  # No need to check the rest once we know it's too close

                if obstacle_detected:
                    print("[WARNING] Obstacle too close! Waiting for clearance...")
                    set_blink(Color(255,0,0), 10)
                    time.sleep(1)
                else:
                    print("[INFO] No nearby obstacles. Proceeding with mission.")
                    set_blink(Color(0,0,255), 5)
                    obstacle_detected = False


            while True:
                user_input = input("Type 'start' to begin mission: ").strip().lower()
                if user_input == "start":
                    break
                else:
                    print("Waiting for user to type 'start'...")
            

            
            # print("[INFO] Priming with dummy setpoints before OFFBOARD...")
            # for _ in range(60):
            #     master.mav.set_position_target_local_ned_send(
            #         int((time.time() - boot_time) * 1e3) % (2**32),
            #         master.target_system,
            #         master.target_component,
            #         mavutil.mavlink.MAV_FRAME_LOCAL_NED,
            #         0b0000111111000111,
            #         0, 0, -targetHoverAlt,   # Position setpoint (e.g., hover 4m above ground)
            #         0, 0, 0,
            #         0, 0, 0,
            #         0, 0
            #     )
            #     time.sleep(0.1)  # 10 Hz
            print("[INFO] Priming with dummy velocity setpoints before OFFBOARD...")
            for _ in range(60):
                master.mav.set_position_target_local_ned_send(
                    int((time.time() - boot_time) * 1e6) % (2**32),
                    master.target_system,
                    master.target_component,
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED,
                    0b0000111111000110,  # Use velocity only
                    0, 0, 0,             # Position ignored
                    0, 0, -1.0,          # Small upward velocity in NED (negative z = up)
                    0, 0, 0,             # Acceleration ignored
                    0, 0                 # Yaw, yaw rate ignored
                )
                time.sleep(0.1)  # 10 Hz


            offboard = OffboardThread(master, z=-targetHoverAlt)
            offboard.start()

            set_offboard_mode(master)
            # if not wait_for_offboard(master) or not listen_for_statustext(master, keyword="offboard"):
            #     print("[ERROR] Offboard mode not confirmed through STATUSTEXT.")
            #     offboard.stop()
            #     offboard.join()
            #     return
                
            # if arm_drone(master):    
            #     takeoff_to_altitude(master, home_z, target_altitude=targetHoverAlt, climb_rate=1)





            #     time.sleep(5)
            #     land_drone(master)
            #     wait_for_landing(master)



            #     stop_blink()
            #     blink_until_solid()
            #     print("Mission completed.")

            #     lidar_thread.stop()
            #     lidar_thread.join()
            #     time.sleep(5)
            #     change_led_color(Color(0,0,0))        
            #     print("Returning to terminal")
            #     sys.exit(0)

                

            #     #     master.mav.command_long_send(
            #     #         master.target_system,
            #     #         master.target_component,
            #     #         mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
            #     #         0,
            #     #         0, 0, 0, 0, 0, 0, 0
            #     #     )

            #     #     print("[INFO] Drone disarmed after landing.")
            # else:
            #     print("[ERROR] Drone failed to arm.")

        else:
            print("[ERROR] Preflight health checks failed. Aborting mission.")
            change_led_color(Color(0,0,0))    
    except KeyboardInterrupt:
        print("\n[INFO] Mission interrupted by user.")
        stop_blink()
        change_led_color(Color(0,0,0))
        lidar_thread.stop()
        lidar_thread.join()        
        sys.exit(0)

if __name__ == "__main__":
    main()
    