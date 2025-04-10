from pymavlink import mavutil
import time
import subprocess  # <-- Needed to launch the lidar script

# Connect to Pixhawk (adjust port if needed)
master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Heartbeat received.")

# Wait for GPS fix
print("Waiting for 3D GPS fix...")
while True:
    msg = master.recv_match(type='GPS_RAW_INT', blocking=True)
    if msg and msg.fix_type >= 3 and msg.satellites_visible >= 6:
        print(f"GPS Fix Acquired: Fix={msg.fix_type}, Satellites={msg.satellites_visible}")
        break
    else:
        print(f"Fix: {msg.fix_type}, Satellites: {msg.satellites_visible}")
        time.sleep(1)

# Ask user for target
target_lat = float(input("Enter target latitude: "))
target_lon = float(input("Enter target longitude: "))
target_alt = float(input("Enter target altitude (meters): "))

# Set mode to GUIDED
def set_mode(mode):
    mode_id = master.mode_mapping()[mode]
    master.mav.set_mode_send(
        master.target_system,
        mavutil.mavlink.MAV_MODE_FLAG_CUSTOM_MODE_ENABLED,
        mode_id)
    ack = False
    while not ack:
        ack_msg = master.recv_match(type='COMMAND_ACK', blocking=True)
        if ack_msg.command == mavutil.mavlink.MAV_CMD_DO_SET_MODE:
            ack = True
    print(f"Mode set to {mode}")

set_mode("GUIDED")

# Arm the drone
def arm_drone():
    print("Arming motors...")
    master.arducopter_arm()
    master.motors_armed_wait()
    print("Motors armed.")

arm_drone()

# Optional: Take off first
def takeoff(alt):
    print(f"Taking off to {alt} meters...")
    master.mav.command_long_send(
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF,
        0,
        0, 0, 0, 0, 0, 0,
        alt
    )
    time.sleep(10)  # Give time to ascend

takeoff(target_alt)

# -----------------------------------------------
# ✅ LAUNCH LIDAR AVOIDANCE SCRIPT HERE
# This allows object avoidance to run in parallel
# Make sure lidar_avoidance.py is in the same folder or use full path
# -----------------------------------------------
lidar_process = subprocess.Popen(['python3', 'lidar_avoidance.py'])
print("LIDAR avoidance script started.")

# Go to target location
def goto_position_target_global_int(lat, lon, alt):
    print(f"Flying to {lat}, {lon}, {alt}m")
    master.mav.set_position_target_global_int_send(
        0,
        master.target_system,
        master.target_component,
        mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
        0b0000111111111000,
        int(lat * 1e7),
        int(lon * 1e7),
        alt,
        0, 0, 0,
        0, 0, 0,
        0, 0
    )

goto_position_target_global_int(target_lat, target_lon, target_alt)
print("Command sent.")

# -----------------------------------------------
# ✅ OPTIONAL: STOP LIDAR SCRIPT AFTER 30 SECONDS
# You could replace this with position check instead
# -----------------------------------------------
time.sleep(30)
lidar_process.terminate()
print("LIDAR avoidance script terminated.")
