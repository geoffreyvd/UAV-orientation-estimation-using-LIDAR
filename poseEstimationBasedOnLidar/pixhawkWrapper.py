#!/usr/bin/env python3
import rospy
#from mavros_msgs.srv import CommandBool, CommandTOL, SetMode
#from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Imu
#from pyquaternion import Quaternion
from math import atan2, pi

#static usb refernce for pixhawp4:
#idVendor: 26ac
#idProduct: 0032

def extractEulerYaw(orient):
    #q0, q1, q2, q3 corresponds to w,x,y,z 
    q = [orient.w, orient.x, orient.y, orient.z]
    return (atan2(2.0 * (q[3] * q[0] + q[1] * q[2]) , - 1.0 + 2.0 * (q[0] * q[0] + q[1] * q[1])))

class pixhawk():
    def __init__(self):
        self.previousYaw = 0
        rospy.init_node('listener', anonymous=True)
        rospy.Subscriber("/mavros/imu/data", Imu, self.callback)

    def callback(self, data):
        # rospy.loginfo("\ndata.orientation:\nx: [{}]\ny: [{}]\nz: [{}]\nw: [{}]"
        # .format(data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w))
        self.orientation = data.orientation

    def getImuYaw(self):
        yaw = extractEulerYaw(self.orientation)
        return yaw
    
    def getImuYawDisplacement(self):
        yaw = extractEulerYaw(self.orientation)
        diffInYaw = self.previousYaw - yaw
        self.previousYaw = yaw
        return diffInYaw

if __name__ == '__main__':
    px = pixhawk()
    rospy.spin()