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
    plt.grid()
    plt.show()

def plotLidar(listOfYaw, listOfAverageYaw, listOfYawImu):
    plt.ion()
    y = np.asarray(listOfYaw)
    # y1 = np.asarray(listOfAverageYaw)
    y2 = np.asarray(listOfYawImu)
    plt.plot(y, label='Lidar yaw')
    # plt.plot(y1)
    plt.plot(y2, label='IMU yaw')
    legend = plt.legend(shadow=True, fontsize='x-large')
    plt.grid()
    plt.xlabel('time (0,1s)')
    plt.ylabel('yaw (degrees)')
    plt.title('Yaw degree over time')
    plt.draw()
    plt.pause(1000)
    plt.clf()

def plotYaw(yawMeasurementsLidar, yawMeasurementsImu, yawMeasurementsEstimate, bias, biascompensated):
    plt.ion()
    y = np.asarray(yawMeasurementsLidar)
    y1 = np.asarray(yawMeasurementsImu)
    y2 = np.asarray(yawMeasurementsEstimate)
    bias = np.asarray(bias)
    biascompensated = np.asarray(biascompensated)
    plt.plot(list(range(0, (len(yawMeasurementsLidar)*5), 5)), y, label='LIDAR yaw')
    plt.plot(y1, label='IMU yaw')
    plt.plot(y2, label='KF estimated yaw')
    plt.plot(bias, label='KF estimated bias')
    plt.plot(biascompensated, label='IMU compensated bias')
    legend = plt.legend(shadow=True, fontsize='x-large')
    plt.grid()
    plt.xlabel('KF iterations ')
    plt.ylabel('yaw (degrees)')
    plt.title('Yaw degree over iterations')
    plt.draw()
    plt.pause(1000)
    plt.clf()    

def plotPosition(x, y):
    plt.ion()
    x = np.asarray(x)
    y = np.asarray(y)
    plt.plot(x, y)
    # plt.plot(y)
    plt.grid()
    plt.xlabel('x position (mm)')
    plt.ylabel('y position (mm)')
    plt.title('x and y position coordinates') 
    plt.axis('equal')
    plt.draw()
    plt.pause(1000)
    plt.clf()
    

if __name__ == '__main__':
    x = [0,1,2,0,1,2]
    y = [1,2,3,4,5, 6]
    plotPosition(x, y)

    # plotImu()
