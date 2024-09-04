# sourced from https://blog.pictor.us/lqr-control-of-a-self-balancing-robot/

import numpy as np
import control

# System Parameters
g = 9.81  # acceleration due to gravity (m/s^2)

# robot physical parameters
# these parameters need to match that included in the robot model
# definition (robot-02.urdf)
m = 0.578   # total mass (kg)
l = 0.023   # length to the center of mass from wheel axis (m)
r = 0.034  # Wheel radius (m)
max_velocity = 1.0  # Maximum velocity corresponding to full stick input
THROTTLE_GAIN = 1.0  # Mapping LQR control output to Throttle input

A = np.array([[0, 1, 0, 0],
              [g/l, 0, 0, 0],
              [0, 0, 0, 1],
              [-g/m, 0, 0, 0]])

B = np.array([[0],
              [- 1 / (m * (l ** 2))],
              [0],
              [ 1 / m]])

# Weighting matrices
Q = np.diag([0.7, 0.0, 0.0, 2.5])  # Penalizing theta, theta_dot, x, and x_dot
R = np.array([[0.5]]) # Penalizing control effort

# Calculate the LQR gain matrix K
# This is what our Python script returns.
K, _, _ = control.lqr(A, B, Q, R)


print(f"LQR_K = [ {K[0][0]}, {K[0][1]}, {K[0][2]}, {K[0][3]}]")

