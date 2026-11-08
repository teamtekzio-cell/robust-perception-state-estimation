"""
test_dead_reckoning.py - Checks for the baselines and the error metrics.
"""
import numpy as np
import pandas as pd
import pytest
from dead_reckoning import odometry_only, odometry_gyro
from metrics import summary, angle_diff


def perfect_straight_sensors(n=501, v=0.2, dt=0.02):
    """Perfect readings for driving straight: both wheels v, gyro 0."""
    return pd.DataFrame({
        "t": np.arange(n) * dt,
        "meas_v_left": np.full(n, v),
        "meas_v_right": np.full(n, v),
        "meas_gyro": np.zeros(n),
    })


def test_perfect_sensors_give_perfect_estimate():
    s = perfect_straight_sensors()
    est = odometry_only(s)
    assert est.x.iloc[-1] == pytest.approx(0.2 * 10.0)   # 500 steps x 0.02 s = 10 s
    assert est.y.iloc[-1] == pytest.approx(0.0)


def test_gyro_bias_causes_heading_drift():
    s = perfect_straight_sensors()
    s["meas_gyro"] = 0.01                                 # bias only
    est = odometry_gyro(s)
    assert est.theta.iloc[-1] == pytest.approx(0.01 * 10.0)  # drift = bias x time
    assert est.y.iloc[-1] > 0                             # path bends left


def test_identical_paths_have_zero_error():
    s = perfect_straight_sensors()
    est = odometry_only(s)
    scores = summary(est, est)
    assert scores["pos_rmse_m"] == 0.0
    assert scores["final_pos_err_m"] == 0.0


def test_angle_diff_wraps_correctly():
    a = np.radians(179.0)
    b = np.radians(-179.0)
    assert np.degrees(angle_diff(a, b)) == pytest.approx(-2.0)