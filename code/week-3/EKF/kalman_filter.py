import numpy as np
from math import sqrt
from math import atan2
from tools import Jacobian

class KalmanFilter:
    def __init__(self, x_in, P_in, F_in, H_in, R_in, Q_in):
        self.x = x_in
        self.P = P_in
        self.F = F_in
        self.H = H_in
        self.R = R_in
        self.Q = Q_in

    def predict(self):
        self.x = np.dot(self.F, self.x)
        self.P = np.dot(np.dot(self.F, self.P), self.F.T) + self.Q

    def update(self, z):
        S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
        K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
        # Calculate new estimates
        self.x = self.x + np.dot(K, z - np.dot(self.H, self.x))
        self.P = self.P - np.dot(np.dot(K, self.H), self.P)

    def update_ekf(self, z):

        px, py, vx, vy = self.x
        Hj = Jacobian(self.x)  # 3X4 matrix

        S = np.dot(np.dot(Hj, self.P), Hj.T) + self.R #3X3 matrix
        K = np.dot(np.dot(self.P, Hj.T), np.linalg.inv(S)) #4X3 matrix

        c1 = px * px + py * py
        c2 = sqrt(c1)
        hx = [c2, atan2(py,px), (px * vx + py * vy)/c2]

        PI = 3.14159;

        y = z - hx

        if (y[1] > PI) :
            y[1] = (y[1]-2*PI)
        if (y[1] < -PI) :
            y[1] = (y[1]+2*PI)

        self.x = self.x + np.dot(K, y)
        self.P = self.P - np.dot(np.dot(K, Hj), self.P)

        # TODO: Implement EKF update for radar measurements
        # 1. Compute Jacobian Matrix Hj
        # 2. Calculate S = Hj * P' * Hj^T + R
        # 3. Calculate Kalman gain K = P' * Hj^T * inv(S)
        # 4. Estimate y = z - h(x')
        # 5. Normalize phi so that it is between -PI and +PI
        # 6. Calculate new estimates
        #    x = x' + K * y
        #    P = (I - K * Hj) * P'
