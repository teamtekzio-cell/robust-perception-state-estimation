"""
test_ekf.py - Checks that the EKF behaves as the theory says.
"""
import numpy as np
import pytest
from ekf import EKF, X, Y, TH, W


def test_predict_moves_robot_and_grows_uncertainty():
    ekf = EKF()
    ekf.s[3] = 0.2                       # v = 0.2 m/s
    p_before = ekf.P[X, X]
    ekf.predict(1.0)
    assert ekf.s[X] == pytest.approx(0.2)   # moved 0.2 m forward
    assert ekf.P[X, X] > p_before           # less certain after predicting


def test_update_shrinks_uncertainty():
    ekf = EKF()
    ekf.predict(1.0)
    p_before = ekf.P[W, W]
    ekf.update_gyro(0.5, np.array([[0.01 ** 2]]))
    assert ekf.P[W, W] < p_before           # more certain after measuring


def test_fusion_trusts_the_more_precise_sensor():
    ekf = EKF()
    ekf.predict(1.0)
    # Noisy odometry says omega = 0.6, precise gyro says 0.5
    ekf.update_odometry(0.0, 0.6, np.diag([1e-4, 0.1 ** 2]))
    ekf.update_gyro(0.5, np.array([[0.01 ** 2]]))
    assert 0.5 < ekf.s[W] < 0.55            # between the two, much closer to the gyro


def test_perfect_measurements_track_a_straight_line():
    ekf = EKF()
    R_odom, R_gyro = np.diag([1e-6, 1e-6]), np.array([[1e-6]])
    for _ in range(500):                    # 10 s at 50 Hz
        ekf.predict(0.02)
        ekf.update_odometry(0.2, 0.0, R_odom)
        ekf.update_gyro(0.0, R_gyro)
    assert ekf.s[X] == pytest.approx(2.0, abs=0.01)
    assert ekf.s[Y] == pytest.approx(0.0, abs=1e-6)
    assert ekf.s[TH] == pytest.approx(0.0, abs=1e-6)