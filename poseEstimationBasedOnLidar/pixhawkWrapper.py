#!/usr/bin/env python3
import rospy
#from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
#from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu
#from pyquaternion import Quaternion
from math import atan2, pi
from time import time, sleep
from multiprocessing import Process, Pipe

#static usb refernce for pixhawp4:
#idVendor: 26ac
#idProduct: 0032

def callbackIMU(data, pipeToMain):
    pipeToMain.send(data.orientation)

#parallel process needed because imu data was not being fetched at 50Hz, because main was blocking thread
def parallelFuncImuYaw(pipeToMain):
    rospy.init_node('listener', anonymous=True)
    #tcp nodelay, don wait for ACK, just send all pakcets individually asap
    rospy.Subscriber("/mavros/imu/data", Imu, callbackIMU, pipeToMain, tcp_nodelay=True)
    while 1:
        sleep(0.01)
    pipeToMain.close()

def extractEulerYaw(orient):
    #q0, q1, q2, q3 corresponds to w,x,y,z 
    q = [orient.w, orient.x, orient.y, orient.z]
    return (atan2(2.0 * (q[3] * q[0] + q[1] * q[2]) , - 1.0 + 2.0 * (q[0] * q[0] + q[1] * q[1])))

class pixhawk():
    def __init__(self):
        self.previousYaw = None
        self.orientation = None
        self.start_sec = time()

        self.pipeFromImu, child_conn = Pipe()
        self.processPipeImuYaw = Process(target=parallelFuncImuYaw, args=(child_conn,))
        self.processPipeImuYaw.start()

        # rospy.Subscriber("/mavros/imu/data", Imu, self.callback, tcp_nodelay=True)

    def callback(self, data):
        # rospy.loginfo("\ndata.orientation:\nx: [{}]\ny: [{}]\nz: [{}]\nw: [{}]"
        # .format(data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w))
        self.orientation = data.orientation
        print("[{}], yeay yaw imu".format(time() - self.start_sec))

    def getImuYaw(self):
        yaw = extractEulerYaw(self.orientation)
        return yaw

    def closeParallelProcess(self):
        self.processPipeImuYaw.terminate()
    
    # def getImuYawDisplacement(self):
    #     if self.orientation is not None:
    #         yaw = extractEulerYaw(self.orientation)
    #         if self.previousYaw is None:
    #             self.previousYaw = yaw
    #             return None
    #         diffInYaw = yaw - self.previousYaw
    #         self.previousYaw = yaw
    #         return diffInYaw
    #     else:
    #         return None
    
    def getImuYawDisplacement(self):
        yaw = None        
        while self.pipeFromImu.poll():
            yaw = extractEulerYaw(self.pipeFromImu.recv())
        if yaw is not None:
            print(yaw*180/pi)
            if self.previousYaw is not None:
                imuYawBetweenLidarScans = yaw - self.previousYaw
                self.previousYaw = yaw
                return imuYawBetweenLidarScans
            self.previousYaw = yaw
        return None

if __name__ == '__main__':
    px = pixhawk()
    rospy.spin()