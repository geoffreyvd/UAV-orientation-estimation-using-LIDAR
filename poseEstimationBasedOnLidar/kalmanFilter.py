#!/usr/bin/env python3
import numpy as np
from filterpy.kalman import KalmanFilter

class KF():
    def __init__(self):
        self.f = KalmanFilter (dim_x=2, dim_z=1, dim_u=1)

        # Assign the initial value for the state 
        self.f.x = np.array([[0.],[0.]]) 

        # Define the state transition matrix:
        self.f.F = np.array([[1.,-0.02],[0.,1.]])

        # Define the measurement function:
        self.f.H = np.array([[1.,0.]])

        # Define the covariance matrix. 
        self.f.P = np.array([[1000.,0.],[0.,1000.]])

        # Now assign the measurement noise. Here the dimension is 1x1, so I can use a scalar
        self.f.R = 0.08539

        # the control matrix, (0,02 = de predict rate van de imu)
        self.f.B = np.array([[0.02],[0.]]) 

        # Finally, I will assign the process noise. Here I will take advantage of another FilterPy library function:
        self.f.Q = 0.000001
        
        self.estimatedStates = []

    def predict(self, u):
        # u = imu
        self.f.predict(u)
        self.estimatedStates.append(self.f.x[0])
        return self.f.x

    def update(self, z):
        #z = lidarYaw
        self.f.update(z)
        self.estimatedStates.append(self.f.x[0])
        return self.f.x

    def getEstimatedStates(self):
        return self.estimatedStates