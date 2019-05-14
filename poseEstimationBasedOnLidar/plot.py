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

def plotImu():
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

def plotLidar(listOfYaw, listOfAverageYaw, listOfYawImu):
    plt.ion()
    y = np.asarray(listOfYaw)
    # y1 = np.asarray(listOfAverageYaw)
    y2 = np.asarray(listOfYawImu)
    plt.plot(y)
    # plt.plot(y1)
    plt.plot(y2)
    plt.draw()
    plt.pause(1000)
    plt.clf()

def plotYaw(yawMeasurementsLidar, yawMeasurementsImu, yawMeasurementsEstimate):
    plt.ion()
    y = np.asarray(yawMeasurementsLidar)
    y1 = np.asarray(yawMeasurementsImu)
    y2 = np.asarray(yawMeasurementsEstimate)
    plt.plot(list(range(0, 606, 6)), y)
    plt.plot(list(range(0, 510)), y1)
    plt.plot(list(range(0, 612)), y2)
    plt.draw()
    plt.pause(100)
    plt.clf()    

if __name__ == '__main__':
    
    plotImu()
