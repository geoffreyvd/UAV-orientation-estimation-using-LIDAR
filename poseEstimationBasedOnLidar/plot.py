#!/usr/bin/env python3
# libraries

import rospy
from sensor_msgs.msg import Imu

from time import sleep
import matplotlib
matplotlib.use("TkAgg")
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
global yaw
yaw = 0
def callbackIMU(data):
    global yaw
    yaw += data.angular_velocity.z

rospy.init_node('listener', anonymous=True)
#tcp nodelay, don wait for ACK, just send all pakcets individually asap
rospy.Subscriber("/mavros/imu/data_raw", Imu, callbackIMU, tcp_nodelay=True)

fig, ax = plt.subplots()
xdata, ydata = [], []
ln, = plt.plot([], [], 'r')
global counter
counter =0

def init():
    ax.set_xlim(0,240)
    ax.set_ylim(-3, 3)
    return ln,

def update(frame):
    global yaw
    xdata.append(frame)
    ydata.append(yaw)
    ln.set_data(xdata, ydata)
    return ln,

ani = FuncAnimation(fig, update, frames=240,
                    init_func=init, interval = 1000)
plt.show()