"""
run_outages.py - Temporary sensor failure experiment (Session 7, brief Experiment 4).

Square trajectory, nominal sensors, 10 seeds. Three scenarios:
    no fault          : all sensors working (reference)
    gyro off 15 s     : gyroscope silent from 20 s to 35 s (includes a turn)
    wheels off 5 s    : wheel encoders silent from 10 s to 15 s (includes a turn)

For each method we measure the overall RMSE and the EXTRA final error
caused by the outage (same seed with vs without the fault).

Run from the project root:
    python src/run_outages.py
"""
import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from sensors import SensorConfig, simulate_sensors
from faults import apply_outage
from dead_reckoning import odometry_only, odometry_gyro
from run_ekf import run_ekf
from metrics import errors_over_time, summary

TRAJECTORY = "square"
SEEDS = list(range(1, 11))
RESULTS_DIR = "experiments/results"
SCENARIOS = {
    "no fault": None,
    "gyro off 20-35 s": ("gyro", 20.0, 35.0),
    "wheels off 10-15 s": ("wheels", 10.0, 15.0),
}
METHODS = ["Odometry only", "Odometry + gyro heading", "EKF (odom + gyro)"]
COLOURS = {"Odometry only": "tab:orange", "Odometry + gyro heading": "tab:green", "EKF (odom + gyro)": "tab:blue"}


def estimate_all(sensors, cfg):
    return {
        "Odometry only": odometry_only(sensors),
        "Odometry + gyro heading": odometry_gyro(sensors),
        "EKF (odom + gyro)": run_ekf(sensors, cfg),
    }


def main():
    cfg = SensorConfig()
    gt = pd.read_csv(f"{RESULTS_DIR}/ground_truth_{TRAJECTORY}.csv")
    rows, seed1 = [], {}

    for seed in SEEDS:
        clean = simulate_sensors(gt, cfg, seed)
        for scenario, fault in SCENARIOS.items():
            sensors = clean if fault is None else apply_outage(clean, *fault)
            for method, est in estimate_all(sensors, cfg).items():
                rows.append({"scenario": scenario, "method": method, "seed": seed, **summary(est, gt)})
                if seed == 1:
                    seed1[(scenario, method)] = est

    df = pd.DataFrame(rows)
    # Extra final error caused by the outage = with fault - without fault (same seed, same method)
    base = df[df.scenario == "no fault"].set_index(["method", "seed"])["final_pos_err_m"]
    df["extra_final_err_m"] = df.apply(lambda r: r.final_pos_err_m - base[(r.method, r.seed)], axis=1)
    df.to_csv(f"{RESULTS_DIR}/session7_outages.csv", index=False)

    pd.set_option("display.width", 140)
    table = df.groupby(["scenario", "method"], sort=False).agg(
        pos_rmse_mean=("pos_rmse_m", "mean"), pos_rmse_std=("pos_rmse_m", "std"),
        extra_final_err_mean=("extra_final_err_m", "mean")).round(3)
    print("=== Square, 10 seeds: effect of sensor outages ===")
    print(table.to_string())

    # ---------- Plots (seed 1) ----------
    fig, axes = plt.subplots(2, 2, figsize=(14, 8))
    for col, scenario in enumerate(["gyro off 20-35 s", "wheels off 10-15 s"]):
        _, t0, t1 = SCENARIOS[scenario]
        ax = axes[0, col]
        for method in METHODS:
            err = errors_over_time(seed1[(scenario, method)], gt)
            ax.plot(err.t, err.pos_err, color=COLOURS[method], label=method)
            ref = errors_over_time(seed1[("no fault", method)], gt)
            ax.plot(ref.t, ref.pos_err, color=COLOURS[method], ls=":", alpha=0.6)
        ax.axvspan(t0, t1, color="red", alpha=0.12, label="sensor outage")
        ax.set_title(f"{scenario}: position error (solid) vs no fault (dotted)")
        ax.set_xlabel("time (s)"); ax.set_ylabel("position error (m)")
        ax.grid(alpha=0.3); ax.legend(fontsize=8)

        ax = axes[1, col]
        ekf = seed1[(scenario, "EKF (odom + gyro)")]
        ekf_ref = seed1[("no fault", "EKF (odom + gyro)")]
        ax.plot(ekf.t, ekf.sigma_pos, color="tab:blue", label="EKF position uncertainty (with outage)")
        ax.plot(ekf_ref.t, ekf_ref.sigma_pos, color="tab:blue", ls=":", label="EKF position uncertainty (no fault)")
        ax.axvspan(t0, t1, color="red", alpha=0.12, label="sensor outage")
        ax.set_title(f"{scenario}: EKF's own position uncertainty (1σ)")
        ax.set_xlabel("time (s)"); ax.set_ylabel("σ position (m)")
        ax.grid(alpha=0.3); ax.legend(fontsize=8)

    plt.tight_layout()
    plt.savefig("evidence/screenshots/session7_outages.png", dpi=150)
    print("\nSaved experiments/results/session7_outages.csv and evidence/screenshots/session7_outages.png")


if __name__ == "__main__":
    main()  