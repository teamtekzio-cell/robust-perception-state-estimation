"""
run_ekf.py - Run the EKF on every trajectory and compare it with the
Session 4 baselines using the same metrics (Session 5).

Run from the project root:
    python src/run_ekf.py
"""
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from ekf import EKF, TH
from sensors import SensorConfig
from dead_reckoning import odometry_only, odometry_gyro, WHEEL_BASE
from metrics import errors_over_time, summary

TRAJECTORIES = ["square", "circle", "figure8"]
SEED = 1
RESULTS_DIR = "experiments/results"
COLOURS = {"Odometry only": "tab:orange", "Odometry + gyro heading": "tab:green", "EKF (odom + gyro)": "tab:blue"}


def measurement_noise(cfg: SensorConfig):
    """R matrices built from the sensor noise levels (Session 3 settings)."""
    var_v = cfg.sigma_wheel ** 2 / 2                       # v = average of two wheels
    var_w = 2 * cfg.sigma_wheel ** 2 / WHEEL_BASE ** 2     # omega = difference / L
    R_odom = np.diag([var_v, var_w])
    R_gyro = np.array([[cfg.sigma_gyro ** 2]])
    return R_odom, R_gyro


def run_ekf(sensors: pd.DataFrame, cfg: SensorConfig) -> pd.DataFrame:
    R_odom, R_gyro = measurement_noise(cfg)
    ekf = EKF()
    t = sensors.t.to_numpy()
    rows = [dict(t=t[0], x=0.0, y=0.0, theta=0.0, sigma_theta=0.0)]
    for k in range(1, len(t)):
        ekf.predict(t[k] - t[k - 1])
        vl, vr = sensors.meas_v_left.iloc[k], sensors.meas_v_right.iloc[k]
        ekf.update_odometry((vr + vl) / 2, (vr - vl) / WHEEL_BASE, R_odom)
        ekf.update_gyro(sensors.meas_gyro.iloc[k], R_gyro)
        rows.append(dict(t=t[k], x=ekf.s[0], y=ekf.s[1], theta=ekf.s[2],
                         sigma_theta=np.sqrt(ekf.P[TH, TH])))
    return pd.DataFrame(rows)


def main():
    cfg = SensorConfig()
    rows = []
    fig, axes = plt.subplots(2, 3, figsize=(15, 8.5))
    ekf_square, gt_square = None, None

    for col, name in enumerate(TRAJECTORIES):
        gt = pd.read_csv(f"{RESULTS_DIR}/ground_truth_{name}.csv")
        sensors = pd.read_csv(f"{RESULTS_DIR}/sensors_{name}_seed{SEED}.csv")
        estimates = {
            "Odometry only": odometry_only(sensors),
            "Odometry + gyro heading": odometry_gyro(sensors),
            "EKF (odom + gyro)": run_ekf(sensors, cfg),
        }
        if name == "square":
            ekf_square, gt_square = estimates["EKF (odom + gyro)"], gt

        ax_path, ax_err = axes[0, col], axes[1, col]
        ax_path.plot(gt.x, gt.y, "k--", lw=1.5, label="ground truth")
        for method, est in estimates.items():
            rows.append({"trajectory": name, "method": method, "seed": SEED, **summary(est, gt)})
            err = errors_over_time(est, gt)
            ax_path.plot(est.x, est.y, color=COLOURS[method], lw=1.3, label=method)
            ax_err.plot(err.t, err.pos_err, color=COLOURS[method], label=method)
        ax_path.set_title(f"{name}: estimated vs true path")
        ax_path.set_xlabel("x (m)"); ax_path.set_ylabel("y (m)")
        ax_path.set_aspect("equal"); ax_path.grid(alpha=0.3); ax_path.legend(fontsize=7)
        ax_err.set_title(f"{name}: position error over time")
        ax_err.set_xlabel("time (s)"); ax_err.set_ylabel("position error (m)")
        ax_err.grid(alpha=0.3); ax_err.legend(fontsize=7)

    plt.tight_layout()
    plt.savefig("evidence/screenshots/session5_ekf_vs_baselines.png", dpi=150)

    # Consistency check: is the EKF's own uncertainty honest?
    err = errors_over_time(ekf_square, gt_square)
    two_sigma = 2 * np.degrees(ekf_square.sigma_theta)
    plt.figure(figsize=(10, 4))
    plt.plot(err.t, np.degrees(err.head_err), label="actual heading error")
    plt.fill_between(err.t, -two_sigma, two_sigma, alpha=0.25, label="EKF's own ±2σ uncertainty")
    plt.title("EKF heading error vs the EKF's own uncertainty (square)")
    plt.xlabel("time (s)"); plt.ylabel("heading error (deg)")
    plt.grid(alpha=0.3); plt.legend()
    plt.tight_layout()
    plt.savefig("evidence/screenshots/session5_ekf_consistency.png", dpi=150)

    results = pd.DataFrame(rows)
    results.to_csv(f"{RESULTS_DIR}/ekf_results_seed{SEED}.csv", index=False)
    pd.set_option("display.width", 130)
    print(results.round(3).to_string(index=False))
    inside = np.mean(np.abs(np.degrees(err.head_err)) <= two_sigma) * 100
    print(f"\nSquare: heading error inside the EKF's ±2σ band {inside:.0f}% of the time (ideal is about 95%)")
    print("Saved table and plots.")


if __name__ == "__main__":
    main()