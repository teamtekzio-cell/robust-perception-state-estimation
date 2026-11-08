"""
test_faults.py - Checks for outage injection and how estimators cope with missing readings.
"""
import numpy as np
import pandas as pd
import pytest
from faults import apply_outage
from dead_reckoning import odometry_only, odometry_gyro
from run_ekf import run_ekf
from sensors import SensorConfig


def straight_sensors(n=501, v=0.2, dt=0.02):
    return pd.DataFrame({
        "t": np.arange(n) * dt,
        "meas_v_left": np.full(n, v),
        "meas_v_right": np.full(n, v),
        "meas_gyro": np.zeros(n),
    })


def test_outage_only_affects_the_chosen_sensor_and_window():
    s = apply_outage(straight_sensors(), "gyro", 2.0, 4.0)
    window = (s.t >= 2.0) & (s.t < 4.0)
    assert s.meas_gyro[window].isna().all()          # silent inside the window
    assert s.meas_gyro[~window].notna().all()        # normal outside it
    assert s.meas_v_left.notna().all()               # other sensors untouched


def test_dead_reckoning_survives_wheel_outage_by_holding_last_value():
    s = apply_outage(straight_sensors(), "wheels", 2.0, 4.0)
    est = odometry_only(s)
    assert not est.x.isna().any()
    assert est.x.iloc[-1] == pytest.approx(2.0)      # constant speed -> holding is exact


def test_gyro_fallback_uses_wheel_turning_rate():
    s = apply_outage(straight_sensors(), "gyro", 0.0, 10.1)   # gyro silent the whole time
    est = odometry_gyro(s)
    assert est.theta.iloc[-1] == pytest.approx(0.0)  # wheels say straight -> no turning


def test_ekf_keeps_running_and_uncertainty_grows_during_outage():
    s = apply_outage(straight_sensors(), "wheels", 4.0, 8.0)
    est = run_ekf(s, SensorConfig())
    assert not est.x.isna().any()
    before = est.sigma_pos[est.t.between(3.5, 3.99)].mean()
    during = est.sigma_pos[est.t.between(7.5, 7.99)].mean()
    assert during > before                            # less certain without the wheels