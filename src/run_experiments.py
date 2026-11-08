"""
run_experiments.py - Repeated, controlled experiments (Session 6).

Experiment A (E1, nominal)      : default sensors, 10 seeds
Experiment B (cause check)      : same, but systematic errors switched OFF (noise only)
Experiment C (E2, noise levels) : noise only, noise multiplied by 0.5x, 1x, 2x, 4x

Every run: generate sensor data with a seed -> run 3 methods -> score vs ground truth.
Results are saved as CSV; the settings used are saved as JSON (repeatability).

Run from the project root (takes a few minutes):
    python src/run_experiments.py
"""
import json
import time
from dataclasses import asdict, replace

import numpy as np
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from sensors import SensorConfig, simulate_sensors
from dead_reckoning import odometry_only, odometry_gyro
from run_ekf import run_ekf
from metrics import errors_over_time, summary

TRAJECTORIES = ["square", "circle", "figure8"]
SEEDS = list(range(1, 11))                  # 10 trials
NOISE_LEVELS = [0.5, 1.0, 2.0, 4.0]         # multipliers on the noise sigmas
RESULTS_DIR = "experiments/results"
METHODS = ["Odometry only", "Odometry + gyro heading", "EKF (odom + gyro)"]

NOMINAL = SensorConfig()                                          # Session 3 defaults
NOISE_ONLY = replace(NOMINAL, wheel_scale_right=0.0, gyro_bias=0.0)  # systematic errors off


def run_one(gt, cfg, seed):
    """One trial: make sensor data, run every method, return one row per method."""
    sensors = simulate_sensors(gt, cfg, seed)
    estimates = {
        "Odometry only": odometry_only(sensors),
        "Odometry + gyro heading": odometry_gyro(sensors),
        "EKF (odom + gyro)": run_ekf(sensors, cfg),
    }
    rows = []
    for method, est in estimates.items():
        row = {"method": method, "seed": seed, **summary(est, gt)}
        if "sigma_theta" in est:   # EKF only: is its own uncertainty honest?
            err = errors_over_time(est, gt)
            row["inside_2sigma_pct"] = 100 * np.mean(np.abs(err.head_err) <= 2 * est.sigma_theta + 1e-9)
        rows.append(row)
    return rows


def run_condition(label, cfg, ground_truths):
    rows = []
    for name, gt in ground_truths.items():
        for seed in SEEDS:
            for r in run_one(gt, cfg, seed):
                rows.append({"condition": label, "trajectory": name, **r})
    return rows


def mean_std_table(df, group_cols):
    g = df.groupby(group_cols, sort=False)["pos_rmse_m"]
    out = g.agg(["mean", "std"]).reset_index()
    out["pos_rmse (mean ± std, m)"] = out.apply(lambda r: f"{r['mean']:.3f} ± {r['std']:.3f}", axis=1)
    return out.drop(columns=["mean", "std"])


def main():
    start = time.time()
    ground_truths = {n: pd.read_csv(f"{RESULTS_DIR}/ground_truth_{n}.csv") for n in TRAJECTORIES}

    # Save the exact settings used, so anyone can repeat the experiments
    with open("experiments/configs/session6_config.json", "w") as f:
        json.dump({"seeds": SEEDS, "trajectories": TRAJECTORIES, "noise_levels": NOISE_LEVELS,
                   "nominal": asdict(NOMINAL), "noise_only": asdict(NOISE_ONLY)}, f, indent=2)

    print("Experiment A: nominal sensors ...")
    exp_a = pd.DataFrame(run_condition("nominal", NOMINAL, ground_truths))
    print("Experiment B: noise only (systematic errors off) ...")
    exp_b = pd.DataFrame(run_condition("noise only", NOISE_ONLY, ground_truths))
    print("Experiment C: noise levels ...")
    rows_c = []
    for level in NOISE_LEVELS:
        cfg = replace(NOISE_ONLY, sigma_wheel=NOISE_ONLY.sigma_wheel * level,
                      sigma_gyro=NOISE_ONLY.sigma_gyro * level)
        for r in run_condition(f"{level}x noise", cfg, ground_truths):
            rows_c.append({"noise_level": level, **r})
    exp_c = pd.DataFrame(rows_c)

    ab = pd.concat([exp_a, exp_b])
    ab.to_csv(f"{RESULTS_DIR}/session6_exp_A_B.csv", index=False)
    exp_c.to_csv(f"{RESULTS_DIR}/session6_exp_C_noise_levels.csv", index=False)

    # ---------- Print summary tables ----------
    pd.set_option("display.width", 140)
    print("\n=== Experiments A and B: position RMSE over 10 seeds ===")
    print(mean_std_table(ab, ["condition", "trajectory", "method"]).to_string(index=False))

    ekf_rows = ab[ab.method == "EKF (odom + gyro)"]
    print("\n=== EKF consistency: % of time heading error inside its own ±2σ (ideal ≈ 95%) ===")
    print(ekf_rows.groupby(["condition", "trajectory"], sort=False)["inside_2sigma_pct"]
          .mean().round(1).to_string())

    print("\n=== Experiment C: position RMSE (m), mean over 3 trajectories x 10 seeds ===")
    print(exp_c.pivot_table(index="noise_level", columns="method", values="pos_rmse_m",
                            aggfunc="mean")[METHODS].round(3).to_string())

    # ---------- Plots ----------
    colours = {"Odometry only": "tab:orange", "Odometry + gyro heading": "tab:green", "EKF (odom + gyro)": "tab:blue"}

    # Plot 1: A vs B bar chart (mean ± std), one panel per trajectory
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5), sharey=False)
    width = 0.25
    for ax, name in zip(axes, TRAJECTORIES):
        for i, method in enumerate(METHODS):
            means, stds = [], []
            for cond in ["nominal", "noise only"]:
                vals = ab[(ab.trajectory == name) & (ab.method == method) & (ab.condition == cond)].pos_rmse_m
                means.append(vals.mean()); stds.append(vals.std())
            ax.bar(np.arange(2) + (i - 1) * width, means, width, yerr=stds, capsize=3,
                   color=colours[method], label=method)
        ax.set_xticks([0, 1]); ax.set_xticklabels(["nominal\n(noise + systematic)", "noise only"])
        ax.set_title(f"{name}: position RMSE, 10 seeds"); ax.set_ylabel("position RMSE (m)")
        ax.grid(axis="y", alpha=0.3)
    axes[0].legend(fontsize=8)
    plt.tight_layout()
    plt.savefig("evidence/screenshots/session6_expA_B.png", dpi=150)

    # Plot 2: noise level sweep
    fig, axes = plt.subplots(1, 3, figsize=(15, 4.5))
    for ax, name in zip(axes, TRAJECTORIES):
        for method in METHODS:
            d = exp_c[(exp_c.trajectory == name) & (exp_c.method == method)]
            g = d.groupby("noise_level")["pos_rmse_m"]
            ax.errorbar(g.mean().index, g.mean().values, yerr=g.std().values, marker="o",
                        capsize=3, color=colours[method], label=method)
        ax.set_xscale("log", base=2); ax.set_xticks(NOISE_LEVELS); ax.set_xticklabels([f"{l}x" for l in NOISE_LEVELS])
        ax.set_title(f"{name}: error vs sensor noise (noise only, 10 seeds)")
        ax.set_xlabel("noise level (multiplier)"); ax.set_ylabel("position RMSE (m)")
        ax.grid(alpha=0.3)
    axes[0].legend(fontsize=8)
    plt.tight_layout()
    plt.savefig("evidence/screenshots/session6_expC_noise_levels.png", dpi=150)

    print(f"\nSaved CSVs, config and plots. Total time: {time.time() - start:.0f} s")


if __name__ == "__main__":
    main()