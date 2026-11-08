"""
test_sensors.py - Checks that the simulated sensors add exactly the errors we configured.
"""
import numpy as np
import pandas as pd
import pytest
from sensors import SensorConfig, simulate_sensors


def make_ground_truth(n=20000):
    """A simple fake ground truth: constant wheel speeds and turning rate."""
    return pd.DataFrame({
        "t": np.arange(n) * 0.02,
        "v_left": np.full(n, 0.2),
        "v_right": np.full(n, 0.2),
        "omega": np.full(n, 0.5),
    })


def test_no_error_settings_give_perfect_readings():
    gt = make_ground_truth(100)
    perfect = SensorConfig(sigma_wheel=0, wheel_scale_right=0, sigma_gyro=0, gyro_bias=0)
    s = simulate_sensors(gt, perfect, seed=1)
    assert np.allclose(s.meas_v_left, gt.v_left)
    assert np.allclose(s.meas_gyro, gt.omega)


def test_gyro_bias_and_noise_match_settings():
    gt = make_ground_truth()
    cfg = SensorConfig(sigma_gyro=0.01, gyro_bias=0.005)
    s = simulate_sensors(gt, cfg, seed=1)
    err = s.meas_gyro - gt.omega
    assert err.mean() == pytest.approx(0.005, abs=0.001)   # bias shows up as the mean error
    assert err.std() == pytest.approx(0.01, rel=0.05)      # noise shows up as the spread


def test_right_wheel_scale_error():
    gt = make_ground_truth()
    cfg = SensorConfig(sigma_wheel=0.0, wheel_scale_right=0.01)
    s = simulate_sensors(gt, cfg, seed=1)
    assert np.allclose(s.meas_v_right, 0.2 * 1.01)          # reads 1% too fast


def test_same_seed_gives_identical_data():
    gt = make_ground_truth(500)
    a = simulate_sensors(gt, SensorConfig(), seed=7)
    b = simulate_sensors(gt, SensorConfig(), seed=7)
    c = simulate_sensors(gt, SensorConfig(), seed=8)
    assert a.equals(b)          # repeatable
    assert not a.equals(c)      # a different seed gives different noise