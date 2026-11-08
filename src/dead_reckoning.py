"""
dead_reckoning.py - Baseline estimators with no correction (Sessions 4 and 7).

Both start from the known start pose and add up measured motion step by step.
    odometry_only : heading change from the wheel encoders
    odometry_gyro : forward speed from the wheels, heading change from the gyroscope

Session 7: missing readings (NaN) are handled with simple fallbacks:
    - wheels missing -> keep using the last valid wheel speeds ("hold last value")
    - gyro missing   -> odometry_gyro falls back to the wheel turning rate
"""
import numpy as np
import pandas as pd
from robot_model import wrap_angle

WHEEL_BASE = 0.160  # m, must match the robot model


def integrate(t, v, omega, x0=0.0, y0=0.0, theta0=0.0):
    """Midpoint pose update over measured v and omega (same maths as Session 2)."""
    n = len(t)
    x, y, theta = np.zeros(n), np.zeros(n), np.zeros(n)
    x[0], y[0], theta[0] = x0, y0, theta0
    for k in range(1, n):
        dt = t[k] - t[k - 1]
        theta_mid = theta[k - 1] + omega[k] * dt / 2.0
        x[k] = x[k - 1] + v[k] * np.cos(theta_mid) * dt
        y[k] = y[k - 1] + v[k] * np.sin(theta_mid) * dt
        theta[k] = wrap_angle(theta[k - 1] + omega[k] * dt)
    return pd.DataFrame({"t": t, "x": x, "y": y, "theta": theta})


def wheel_motion(sensors: pd.DataFrame):
    """v and omega from the wheels; missing readings are filled with the last valid value."""
    vl = sensors.meas_v_left.ffill().fillna(0.0).to_numpy()
    vr = sensors.meas_v_right.ffill().fillna(0.0).to_numpy()
    return (vr + vl) / 2.0, (vr - vl) / WHEEL_BASE


def odometry_only(sensors: pd.DataFrame) -> pd.DataFrame:
    v, omega = wheel_motion(sensors)
    return integrate(sensors.t.to_numpy(), v, omega)


def odometry_gyro(sensors: pd.DataFrame) -> pd.DataFrame:
    v, omega_wheels = wheel_motion(sensors)
    gyro = sensors.meas_gyro.to_numpy()
    omega = np.where(np.isnan(gyro), omega_wheels, gyro)   # fallback when gyro is silent
    return integrate(sensors.t.to_numpy(), v, omega)