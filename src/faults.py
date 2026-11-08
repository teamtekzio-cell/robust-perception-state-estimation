"""
faults.py - Inject sensor faults into sensor data (Session 7).

An outage means the sensor sends NO readings for a period.
Missing readings are stored as NaN ("Not a Number"), which is how
the estimators know a reading is absent rather than zero.
"""
import numpy as np
import pandas as pd

SENSOR_COLUMNS = {
    "gyro": ["meas_gyro"],
    "wheels": ["meas_v_left", "meas_v_right"],
}


def apply_outage(sensors: pd.DataFrame, sensor: str, t_start: float, t_end: float) -> pd.DataFrame:
    """Return a copy of the sensor data with one sensor silent between t_start and t_end."""
    faulty = sensors.copy()
    window = (faulty.t >= t_start) & (faulty.t < t_end)
    for col in SENSOR_COLUMNS[sensor]:
        faulty.loc[window, col] = np.nan
    return faulty