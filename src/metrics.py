"""
metrics.py - Compare an estimated path with the ground truth (Session 4).
Every method in the project is scored with these same functions.
"""
import numpy as np
import pandas as pd


def angle_diff(a, b):
    """Smallest signed difference between two angle arrays, in -pi..pi."""
    return np.arctan2(np.sin(a - b), np.cos(a - b))


def errors_over_time(est: pd.DataFrame, gt: pd.DataFrame) -> pd.DataFrame:
    pos_err = np.sqrt((est.x - gt.x) ** 2 + (est.y - gt.y) ** 2)
    head_err = angle_diff(est.theta.to_numpy(), gt.theta.to_numpy())
    return pd.DataFrame({"t": gt.t, "pos_err": pos_err, "head_err": head_err})


def summary(est: pd.DataFrame, gt: pd.DataFrame) -> dict:
    e = errors_over_time(est, gt)
    return {
        "pos_rmse_m": float(np.sqrt(np.mean(e.pos_err ** 2))),
        "head_rmse_deg": float(np.degrees(np.sqrt(np.mean(e.head_err ** 2)))),
        "final_pos_err_m": float(e.pos_err.iloc[-1]),
        "max_pos_err_m": float(e.pos_err.max()),
    }