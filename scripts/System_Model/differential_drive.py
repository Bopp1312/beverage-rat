#!/usr/bin/python3

import numpy as np
import time

def loop():
    # Generate state input for u_w, u_phi

    r = 30/1000 # Wheel radius in Meters 
    l = 400/1000 # Base width in Meters

    u_w = 1.0
    u_phi = 2*np.pi/5
    U = np.matrix([[u_w],
                   [u_phi]])

    # Set Inital conditions
    x = 0
    y = 0
    theta = 0
    x_dot = 0
    y_dot = 0
    theta_dot = 0

    # Define State transition matrix
    freq = 30
    dt = 1/freq
    A = np.eye(3)
    X = np.matrix([[x],
                   [y],
                   [theta]])
    print(X[2])
    t = 0
    while True:
        t_n = t + dt

        # Update B matrix since its non-linear
        B = np.matrix([[np.cos(X[2,0])*dt, 0],
                       [np.sin(X[2,0])*dt, 0],
                       [              0,dt]])
        # Simulate dynamics
        X_n = A*X+B*U
        print("Time: " + str(t_n))
        print(X_n)
        X = X_n
        t = t_n

        if t > 5.0:
            break

if __name__ == "__main__":
    loop()
    print("Finished with simulation!")