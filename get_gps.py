from pymavlink import mavutil

# Connect to MAVProxy via AMA0 serial port
master = mavutil.mavlink_connection('/dev/ttyAMA0', baud=115200)

# Wait for GPS data
print("Waiting for GPS data...")
while True:
    msg = master.recv_match(type='GLOBAL_POSITION_INT', blocking=True)
    if msg:
        lat = msg.lat / 1e7  # Convert to decimal degrees
        lon = msg.lon / 1e7
        alt = msg.alt / 1000  # Convert to meters
        print(f"Latitude: {lat}, Longitude: {lon}, Altitude: {alt}m")
        break
