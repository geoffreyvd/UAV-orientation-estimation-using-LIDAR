#!/usr/bin/env python3
import numpy as np
from filterpy.kalman import KalmanFilter
from math import pi

class KF():
    def __init__(self):
        self.f = KalmanFilter (dim_x=2, dim_z=1, dim_u=1)

        # Assign the initial value for the state 
        self.f.x = np.array([[0.],[0.]]) 

        # Define the state transition matrix:
        self.f.F = np.array([[1.,-0.02],[0.,1.]])

        # Define the measurement function:
        self.f.H = np.array([[1.,0.]])

        # Define the covariance matrix (estimate uncertainty) 
        # (covariance van de begin estimate, dus als de begin yaw een sd van 3 graden heeft of 0,06 radialen:)
        self.f.P = np.array([[0.0036,0.],[0.,1e-5]])
        #0.0000039
        #bias uncertainty moet kleiner zijn dan imu uncertainty 

        # Now assign the measurement noise. Here the dimension is 1x1, so I can use a scalar
        self.f.R = 0.0023788794712703 #0.08539

        # the control matrix, (0,02 = de predict rate van de imu) (voor nu 0,1)
        self.f.B = np.array([[0.02],[0.]]) 

        # Finally, I will assign the process noise. Here I will take advantage of another FilterPy library function:
        self.f.Q = np.array([[1e-6,0.],[0.,1e-8]])
        
        self.estimatedStates = []
        self.estimatedStatesBias = []

    def predict(self, u):
        # u = imu
        self.f.predict(u)
        yaw = self.f.x[0]/pi*180
        bias = self.f.x[1]/pi*180
        self.estimatedStates.append(yaw)
        self.estimatedStatesBias.append(bias)
        return (yaw,bias)

    def update(self, z):
        #z = lidarYaw
        self.f.update(z)
        self.estimatedStates[-1] = self.f.x[0]/pi*180
        self.estimatedStatesBias[-1] =self.f.x[1]/pi*180
        # self.estimatedStates.append(self.f.x[0]/3.1416*180)
        # self.estimatedStatesBias.append(self.f.x[1])
        return self.f.x

    def getEstimatedStates(self):
        return self.estimatedStates

    def getEstimatedStatesBias(self):
        return self.estimatedStatesBias