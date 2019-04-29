#!/usr/bin/env python3
import rospy
#from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
#from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu
#from pyquaternion import Quaternion
from math import atan2, pi
from time import time, sleep
from multiprocessing import Process, Pipe
import matplotlib.pyplot as plt
import numpy as np

READ_FROM_SERIAL = 1
READ_FROM_FILE = 2

def callbackIMU(data, pipeToMain):
    global yaw
    yaw += data.angular_velocity.z*0.02
    pipeToMain.send(yaw)

#parallel process needed because imu data was not being fetched at 50Hz, because main was blocking thread
def parallelFuncImuYaw(pipeToMain):
    global yaw
    yaw = 0
    rospy.init_node('listener', anonymous=True)
    #tcp nodelay, don wait for ACK, just send all pakcets individually asap
    rospy.Subscriber("/mavros/imu/data_raw", Imu, callbackIMU, pipeToMain, tcp_nodelay=True)
    while 1:
        sleep(0.01)
    pipeToMain.close()

# def extractEulerYaw(orient):
#     #q0, q1, q2, q3 corresponds to w,x,y,z 
#     q = [orient.w, orient.x, orient.y, orient.z]
#     return (atan2(2.0 * (q[3] * q[0] + q[1] * q[2]) , - 1.0 + 2.0 * (q[0] * q[0] + q[1] * q[1])))

class pixhawk():
    def __init__(self, readFrom):
        self.previousYaw = None
        self.orientation = None
        self.start_sec = time()

        self.readFrom = readFrom
        if readFrom == READ_FROM_SERIAL:
            self.pipeFromImu, child_conn = Pipe()
            self.processPipeImuYaw = Process(target=parallelFuncImuYaw, args=(child_conn,))
            self.processPipeImuYaw.start()            

    def closeParallelProcess(self):
        if self.readFrom == READ_FROM_SERIAL:
            self.processPipeImuYaw.terminate()
    
    def getImuYawDisplacement(self):
        if self.readFrom == READ_FROM_SERIAL:
            yaw = None        
            while self.pipeFromImu.poll():
                yaw = self.pipeFromImu.recv()
            if yaw is not None:
                if self.previousYaw is not None:
                    imuYawBetweenLidarScans = yaw - self.previousYaw
                    self.previousYaw = yaw
                    return imuYawBetweenLidarScans
                self.previousYaw = yaw
            return None
        else:
            return 0

    def getYawSum(self):
        return self.previousYaw

if __name__ == '__main__':
    px = pixhawk()
    rospy.spin()