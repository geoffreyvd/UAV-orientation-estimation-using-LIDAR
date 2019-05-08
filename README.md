# Indoor UAV Pose estimation based on a 2D lidar, IMU and a 1D lidar. By using novel and efficient algorithms.
Forked from BreezyLidar - Simple, efficient, Lidar access for Linux computers in Python and C++
Pose estimation build on top

![gif](var/output.gif)

# Installation:
First install breezyLidar
then configure static usb ports: lidar to /dev/URG04LX and pixhawk4 to /dev/pixhawk4 check info below

# Run:
first run px4.launch:
roslaunch mavros px4.launch fcu_url:=/dev/pixhawk4:57600  gcs_url:=udp://@localhost:14550
check if u see imu data incoming:
rostopic echo /mavros/imu/data
then run:
./poseEstimationBasedOnLIDAR.py

NOTE, if this programs returns breezylidar connect error, then just retry 2 times

# static usb info:
    static usb refernce for pixhawp4:
        idVendor: 26ac
        idProduct: 0032
    static usb refernece for URG04LX:
        idVendor: 15d1
        idProduct: 0000