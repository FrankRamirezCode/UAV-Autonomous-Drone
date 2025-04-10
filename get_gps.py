from pymavlink import mavutil

# Connect to the Pixhawk
master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=57600)
print("Waiting for heartbeat...")
master.wait_heartbeat()
print("Heartbeat received.")

# Watch for GPS-related messages
print("Waiting for GPS data...")
while True:
    msg = master.recv_match(blocking=True)
    if not msg:
        continue

    mtype = msg.get_type()
    
    if mtype == 'GLOBAL_POSITION_INT':
        lat = msg.lat / 1e7
        lon = msg.lon / 1e7
        alt = msg.alt / 1000.0
        print(f"[GLOBAL_POSITION_INT] Lat: {lat}, Lon: {lon}, Alt: {alt} m")
        break
    elif mtype == 'GPS_RAW_INT':
        fix_type = msg.fix_type
        num_sat = msg.satellites_visible
        print(f"[GPS_RAW_INT] Fix: {fix_type}, Satellites: {num_sat}")
        if fix_type >= 3:  # 3 = 3D Fix
            lat = msg.lat / 1e7
            lon = msg.lon / 1e7
            alt = msg.alt / 1000.0
            print(f"Lat: {lat}, Lon: {lon}, Alt: {alt} m")
            break 
