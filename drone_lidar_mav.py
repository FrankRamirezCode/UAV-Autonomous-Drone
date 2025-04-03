import os
import ydlidar
import time
from pymavlink import mavutil

def Lidar_Init():
    ydlidar.os_init();
    laser = ydlidar.CYdLidar();
    ports = ydlidar.lidarPortList();
    port = "/dev/ydlidar";
    for key, value in ports.items():
        port = value;
    
    # Properties from SDK pg. 87
    laser.setlidaropt(ydlidar.LidarPropSerialPort, port);
    laser.setlidaropt(ydlidar.LidarPropSerialBaudrate, 230400);
    laser.setlidaropt(ydlidar.LidarPropLidarType, ydlidar.TYPE_TRIANGLE);
    laser.setlidaropt(ydlidar.LidarPropDeviceType, ydlidar.YDLIDAR_TYPE_SERIAL);
    laser.setlidaropt(ydlidar.LidarPropScanFrequency, 12.0);
    laser.setlidaropt(ydlidar.LidarPropSampleRate, 9);
    laser.setlidaropt(ydlidar.LidarPropSingleChannel, False);
    laser.setlidaropt(ydlidar.LidarPropMaxAngle, 180.0);
    laser.setlidaropt(ydlidar.LidarPropMinAngle, -180.0);
    laser.setlidaropt(ydlidar.LidarPropMaxRange, 16.0);
    laser.setlidaropt(ydlidar.LidarPropMinRange, 0.01); 



    ret = laser.initialize();
    if ret:
        ret = laser.turnOn();
        scan = ydlidar.LaserScan()
        while ret and ydlidar.os_isOk() :
            r = laser.doProcessSimple(scan);
            if r:
                print("Scan received[",scan.stamp,"]:",scan.points.size());
                #,"ranges is [",1.0/scan.config.scan_time,"]Hz");
            else :
                print("Failed to get Lidar Data.")
        laser.turnOff();
    laser.disconnecting();
    
def Mav_Init():
	master = mavutil.mavlink_connection("/dev/ttyAMA0", baud = 115200)
	master.wait_heartbeat()
	

Lidar_Init()
