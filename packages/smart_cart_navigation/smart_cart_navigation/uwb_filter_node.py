import numpy as np

class Kalman2D:
    def __init__(self):
        # state: [x, y]
        self.x = np.zeros((2, 1))

        # uncertainty (start high)
        self.P = np.eye(2) * 1000

        # process noise (movement smoothness)
        self.Q = np.eye(2) * 0.05

        # measurement noise (UWB noise ~0.08m)
        self.R = np.eye(2) * 0.3

        # measurement model
        self.H = np.eye(2)

    def update(self, z):
        # z = [x, y]
        z = np.array(z).reshape(2, 1)

        # prediction
        P_pred = self.P + self.Q

        # update
        y = z - self.H @ self.x
        S = self.H @ P_pred @ self.H.T + self.R
        K = P_pred @ self.H.T @ np.linalg.inv(S)

        self.x = self.x + K @ y
        self.P = (np.eye(2) - K @ self.H) @ P_pred

        return self.x.flatten()