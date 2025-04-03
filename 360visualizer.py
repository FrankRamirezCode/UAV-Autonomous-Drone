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

fig = plt.figure()
lidar_polar = plt.subplot(polar=True)
lidar_polar.set_rmax(RMAX)
lidar_polar.grid(True)

ports = ydlidar.lidarPortList()
if not ports:
    print("No YDLIDAR ports found!")
    sys.exit(1)
port = next((v for v in ports.values() if "ttyUSB" in v), list(ports.values())[0])
print(f"Using port: {port}")

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
laser.setlidaropt(ydlidar.LidarPropIntenstiy, True)

scan = ydlidar.LaserScan()

# Create a persistent scatter object
scatter = lidar_polar.scatter([], [], c=[], cmap='hsv', alpha=0.95)

def animate(num):
    if laser.doProcessSimple(scan):
        angles = np.array([p.angle for p in scan.points])
        ranges = np.array([p.range for p in scan.points])
        intensity = np.array([p.intensity for p in scan.points])
        coords = np.column_stack((angles, ranges))
        scatter.set_offsets(coords)
        scatter.set_array(intensity)
        return scatter,

if laser.initialize():
    if laser.turnOn():
        try:
            ani = animation.FuncAnimation(fig, animate, interval=50, blit=True)
            plt.show()
        except KeyboardInterrupt:
            print("Interrupted.")
        finally:
            laser.turnOff()
            laser.disconnecting()
            print("LIDAR stopped.")
    else:
        print("Failed to turn on LIDAR.")
else:
    print("Failed to initialize LIDAR.")
