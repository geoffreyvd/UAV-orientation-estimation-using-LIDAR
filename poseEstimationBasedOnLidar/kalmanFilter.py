#!/usr/bin/env python3
# First construct the object with the required dimensionality.

from filterpy.kalman import KalmanFilter
f = KalmanFilter (dim_x=2, dim_z=1)
# Assign the initial value for the state (position and velocity). You can do this with a two dimensional array like so:

f.x = np.array([[2.],    # position
                [0.]])   # velocity
# or just use a one dimensional array, which I prefer doing.

f.x = np.array([2., 0.])
# Define the state transition matrix:

f.F = np.array([[1.,1.],
                [0.,1.]])
# Define the measurement function:

f.H = np.array([[1.,0.]])
# Define the covariance matrix. Here I take advantage of the fact that P already contains np.eye(dim_x), and just multiply by the uncertainty:

f.P *= 1000.
# I could have written:

f.P = np.array([[1000.,    0.],
                [   0., 1000.] ])
# You decide which is more readable and understandable.

# Now assign the measurement noise. Here the dimension is 1x1, so I can use a scalar

f.R = 5
# I could have done this instead:

f.R = np.array([[5.]])
# Note that this must be a 2 dimensional array, as must all the matrices.

# Finally, I will assign the process noise. Here I will take advantage of another FilterPy library function:

from filterpy.common import Q_discrete_white_noise
f.Q = Q_discrete_white_noise(dim=2, dt=0.1, var=0.13)
# Now just perform the standard predict/update loop:

while some_condition_is_true:

z = get_sensor_reading()
f.predict()
f.update(z)

# do_something_with_estimate (f.x)