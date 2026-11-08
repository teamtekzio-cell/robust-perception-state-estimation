"""
ekf.py - Extended Kalman Filter fusing wheel odometry and gyroscope (Session 5).

State  s = [x, y, theta, v, omega]
Predict: constant-velocity motion model (the Session 2 pose update)
Update 1: wheel odometry measures [v, omega]
Update 2: gyroscope measures [omega]
"""
import numpy as np
from robot_model import wrap_angle

X, Y, TH, V, W = 0, 1, 2, 3, 4   # index of each state variable


class EKF:
    def __init__(self, q_pos=1e-6, q_theta=1e-6, q_v=0.05, q_omega=0.5):
        self.s = np.zeros(5)                 # state estimate (start pose known: 0, 0, 0)
        self.P = np.eye(5) * 1e-6            # uncertainty: start pose known very precisely
        # Process noise per second: how much the real motion may differ from the model
        self.q = np.array([q_pos, q_pos, q_theta, q_v, q_omega])

    # ---------------- PREDICT ----------------
    def predict(self, dt):
        x, y, th, v, w = self.s
        # 1. Move the state forward with the motion model f(s)
        self.s[X] = x + v * np.cos(th) * dt
        self.s[Y] = y + v * np.sin(th) * dt
        self.s[TH] = wrap_angle(th + w * dt)
        # v and omega are assumed constant over one small step

        # 2. Jacobian F = df/ds (how each new value depends on each old value)
        F = np.eye(5)
        F[X, TH] = -v * np.sin(th) * dt
        F[X, V] = np.cos(th) * dt
        F[Y, TH] = v * np.cos(th) * dt
        F[Y, V] = np.sin(th) * dt
        F[TH, W] = dt

        # 3. Uncertainty grows: P = F P F^T + Q
        Q = np.diag(self.q * dt)
        self.P = F @ self.P @ F.T + Q

    # ---------------- UPDATE ----------------
    def update(self, z, H, R):
        """Correct the state with a measurement z = H s + noise (noise covariance R)."""
        y = z - H @ self.s                      # innovation (the surprise)
        S = H @ self.P @ H.T + R                # expected size of the surprise
        K = self.P @ H.T @ np.linalg.inv(S)     # Kalman gain (how much to trust z)
        self.s = self.s + K @ y                 # corrected state
        self.s[TH] = wrap_angle(self.s[TH])
        self.P = (np.eye(5) - K @ H) @ self.P   # uncertainty shrinks
        return y, S

    def update_odometry(self, v_meas, omega_meas, R):
        H = np.zeros((2, 5)); H[0, V] = 1; H[1, W] = 1
        return self.update(np.array([v_meas, omega_meas]), H, R)

    def update_gyro(self, omega_meas, R):
        H = np.zeros((1, 5)); H[0, W] = 1
        return self.update(np.array([omega_meas]), H, R)