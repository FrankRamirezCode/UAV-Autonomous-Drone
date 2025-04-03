# import os
# import ydlidar
# import time
# import sys
# from matplotlib.patches import Arc
# import matplotlib.pyplot as plt
# import matplotlib.animation as animation
# import numpy as np

# RMAX = 32.0


# fig = plt.figure()
# lidar_polar = plt.subplot(polar=True)
# lidar_polar.autoscale_view(True,True,True)
# lidar_polar.set_rmax(RMAX)
# lidar_polar.grid(True)
# ports = ydlidar.lidarPortList();
# port = "/dev/ydlidar";
# for key, value in ports.items():
#     port = value;
    
# laser = ydlidar.CYdLidar();
# laser.setlidaropt(ydlidar.LidarPropSerialPort, port);
# laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 230400);
# laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE);
# laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL);
# laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0);
# laser.setlidaropt(ydlidar.LidarPropSampleRate, 4);
# laser.setlidaropt(ydlidar.LidarPropSingleChannel, False);
# laser.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0);
# laser.setlidaropt(ydlidar.LidarPropMinAngle, -180.0);
# laser.setlidaropt(ydlidar.LidarPropMaxRange, 16.0);
# laser.setlidaropt(ydlidar.LidarPropMinRange, 0.02);
# laser.setlidaropt(ydlidar.LidarPropIntenstiy, True);

# scan = ydlidar.LaserScan()

# def animate(num):
    
#     r = laser.doProcessSimple(scan);
#     if r:
#         angle = []
#         ran = []
#         intensity = []
#         for point in scan.points:
#             angle.append(point.angle);
#             ran.append(point.range);
#             intensity.append(point.intensity);
#         lidar_polar.clear()
#         lidar_polar.scatter(angle, ran, c=intensity, cmap='hsv', alpha=0.95)

# ret = laser.initialize();
# if ret:
#     ret = laser.turnOn();
#     if ret:
#         ani = animation.FuncAnimation(fig, animate, interval=50)
#         plt.show()
#     laser.turnOff();
# laser.disconnecting();
# plt.close();
import ydlidar
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import numpy as np
import sys

RMAX = 16.0

# Set up the polar plot
fig = plt.figure()
lidar_polar = plt.subplot(polar=True)
lidar_polar.set_rmax(RMAX)
lidar_polar.grid(True)

# Detect available ports
ports = ydlidar.lidarPortList()
if not ports:
    print("No YDLIDAR ports found!")
    sys.exit(1)

# Prefer port with "ttyUSB", or just pick first available
port = next((v for v in ports.values() if "ttyUSB" in v), list(ports.values())[0])
print(f"Using port: {port}")

# Set up LIDAR
laser = ydlidar.CYdLidar()
laser.setlidaropt(ydlidar.LidarPropSerialPort, port)
laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 230400)
laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE)
laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL)
laser.setlidaropt(ydlidar.LidarPropScanFrequency, 10.0)
laser.setlidaropt(ydlidar.LidarPropSampleRate, 4)
laser.setlidaropt(ydlidar.LidarPropSingleChannel, False)
laser.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0)
laser.setlidaropt(ydlidar.LidarPropMinAngle, -180.0)
laser.setlidaropt(ydlidar.LidarPropMaxRange, RMAX)
laser.setlidaropt(ydlidar.LidarPropMinRange, 0.02)
# Correct typo version from SDK
laser.setlidaropt(ydlidar.LidarPropIntenstiy, True)

scan = ydlidar.LaserScan()

# Live update function for animation
def animate(num):
    if laser.doProcessSimple(scan):
        angle = [p.angle for p in scan.points]
        ran = [p.range for p in scan.points]
        intensity = [p.intensity for p in scan.points]
        lidar_polar.clear()
        lidar_polar.set_rmax(RMAX)
        lidar_polar.scatter(angle, ran, c=intensity, cmap='hsv', alpha=0.9)
        lidar_polar.grid(True)

# Initialize and run laser
if laser.initialize():
    if laser.turnOn():
        print("YDLIDAR is running...")
        try:
            ani = animation.FuncAnimation(fig, animate, interval=50)
            plt.show()
        except KeyboardInterrupt:
            print("Interrupted.")
        finally:
            laser.turnOff()
            laser.disconnecting()
            print("LIDAR stopped.")
    else:
        print("Failed to turn on YDLIDAR.")
else:
    print("Failed to initialize YDLIDAR.")
