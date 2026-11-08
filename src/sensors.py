"""
sensors.py - Simulated wheel encoders and IMU gyroscope (Session 3).

Each reading = true value + systematic error + random noise.
All error settings live in SensorConfig, so later experiments can
change them (e.g. raise the noise) without touching any other code.
"""
from dataclasses import dataclass
import numpy as np
import pandas as pd


@dataclass
class SensorConfig:
    # Wheel encoders (odometry)
    sigma_wheel: float = 0.005        # m/s, random noise on each wheel speed
    wheel_scale_right: float = 0.01   # +1%: right wheel reads 1% too fast (calibration error)
    # IMU gyroscope
    sigma_gyro: float = 0.01          # rad/s, random noise on the turning rate
    gyro_bias: float = 0.005          # rad/s, constant offset added to every reading


def simulate_sensors(ground_truth: pd.DataFrame, config: SensorConfig, seed: int) -> pd.DataFrame:
    """Create noisy sensor readings from a ground-truth table."""
    rng = np.random.default_rng(seed)
    n = len(ground_truth)

    true_v_left = ground_truth["v_left"].to_numpy()
    true_v_right = ground_truth["v_right"].to_numpy()
    true_omega = ground_truth["omega"].to_numpy()

    # Wheel encoders: (scale error) * true speed + noise
    meas_v_left = true_v_left + rng.normal(0, config.sigma_wheel, n)
    meas_v_right = (1 + config.wheel_scale_right) * true_v_right + rng.normal(0, config.sigma_wheel, n)

    # Gyroscope: true turning rate + bias + noise
    meas_gyro = true_omega + config.gyro_bias + rng.normal(0, config.sigma_gyro, n)

    return pd.DataFrame({
        "t": ground_truth["t"].to_numpy(),
        "meas_v_left": meas_v_left,
        "meas_v_right": meas_v_right,
        "meas_gyro": meas_gyro,
    })