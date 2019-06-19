# Novel alghorithm for autonomous indoor UAV orientation and position estimation based on a 2D lidar, IMU and a 1D lidar. 

## Main contributions:
-Efficient line extraction alghorithm (split-and-merge) based on 2D LIDAR
-Filters for walls selection (line filtering) and wall matching
-Accurate yaw estimation
-Frist draft of position estimation
-Kalman filter that is able to 1) estimate the IMU bias and 2) to smoothen the LIDAR estimate 

![](var/output.gif)

## Setup
This algorithm can either be tested with a real LIDAR and Pixhawk 4 or simply be tested with pre-recorded data avaible in this repo.
In both cases BreezyLidar has to be installed: https://github.com/simondlevy/BreezyLidar (Linux only)
Then go to the algorithm directory:
    cd ./poseEstimationBasedOnLidar

### Test with pre-recorded data
Open the main code file (poseEstimationBasedOnLIDAR.py) in your favorite text editor:
    xdg-open poseEstimationBasedOnLIDAR.py
Then make sure URGmocker and pixhawk4 (lines 60 and 61) are constructed with the argument "READ_FROM_FILE", like this:
    self.mocker = URGMocker(READ_FROM_FILE)
    self.pixhawk4 = pixhawk(READ_FROM_FILE)
Then run the program:
    ./poseEstimationBasedOnLidar

### Testing with real hardware (Hakuyo URG04LX and pixhawk)
#### 1. Install ROS
http://wiki.ros.org/kinetic/Installation/Ubuntu
I did those steps on ubuntu 16.04:
sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
sudo apt-get update
sudo apt-get install ros-kinetic-desktop-full
sudo rosdep init
rosdep update
echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
source ~/.bashrc
sudo apt install python-rosinstall python-rosinstall-generator python-wstool build-essential

#### 2. Install mavROS
Simplest install is for Ubuntu, https://dev.px4.io/en/ros/mavros_installation.html
    sudo apt-get install ros-kinetic-mavros ros-kinetic-mavros-extras
    wget https://raw.githubusercontent.com/mavlink/mavros/master/mavros/scripts/install_geographiclib_datasets.sh
./install_geographiclib_datasets.sh

#### 3. Configure static usb ports
First configure static usb ports for the hardware devices: lidar to /dev/URG04LX and pixhawk4 to /dev/pixhawk4 , use these IDS:
    static usb reference for pixhawp4:
        idVendor: 26ac
        idProduct: 0032
    static usb reference for URG04LX:
        idVendor: 15d1
        idProduct: 0000
Alternatively you could edit the code to just use the dynamic assigned usb names (like /deb/USBtty0) to the pixhawk or LIDAR

#### 4. Let's run it
run px4.launch:
    roslaunch mavros px4.launch fcu_url:=/dev/pixhawk4:57600  gcs_url:=udp://@localhost:14550
check if u see imu data incoming:
    rostopic echo /mavros/imu/data
then run:
    ./poseEstimationBasedOnLIDAR.py

NOTE, if this programs returns breezylidar connect error, then just retry 2 times