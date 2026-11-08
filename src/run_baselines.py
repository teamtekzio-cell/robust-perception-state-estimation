"""
run_baselines.py - Run both dead-reckoning baselines on every trajectory,
score them against ground truth and save a results table and plots (Session 4).

Run from the project root:
    python src/run_baselines.py
"""
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from dead_reckoning import odometry_only, odometry_gyro
from metrics import errors_over_time, summary

TRAJECTORIES = ["square", "circle", "figure8"]
SEED = 1
RESULTS_DIR = "experiments/results"
METHODS = {"Odometry only": odometry_only, "Odometry + gyro heading": odometry_gyro}
COLOURS = {"Odometry only": "tab:orange", "Odometry + gyro heading": "tab:green"}


def main():
    rows = []
    fig, axes = plt.subplots(2, 3, figsize=(15, 8.5))

    for col, name in enumerate(TRAJECTORIES):
        gt = pd.read_csv(f"{RESULTS_DIR}/ground_truth_{name}.csv")
        sensors = pd.read_csv(f"{RESULTS_DIR}/sensors_{name}_seed{SEED}.csv")

        ax_path, ax_err = axes[0, col], axes[1, col]
        ax_path.plot(gt.x, gt.y, "k--", lw=1.5, label="ground truth")

        for method_name, method in METHODS.items():
            est = method(sensors)
            scores = summary(est, gt)
            rows.append({"trajectory": name, "method": method_name, "seed": SEED, **scores})

            err = errors_over_time(est, gt)
            ax_path.plot(est.x, est.y, color=COLOURS[method_name], lw=1.5, label=method_name)
            ax_err.plot(err.t, err.pos_err, color=COLOURS[method_name], label=method_name)

        ax_path.set_title(f"{name}: estimated vs true path")
        ax_path.set_xlabel("x (m)"); ax_path.set_ylabel("y (m)")
        ax_path.set_aspect("equal"); ax_path.grid(alpha=0.3); ax_path.legend(fontsize=8)
        ax_err.set_title(f"{name}: position error over time")
        ax_err.set_xlabel("time (s)"); ax_err.set_ylabel("position error (m)")
        ax_err.grid(alpha=0.3); ax_err.legend(fontsize=8)

    plt.tight_layout()
    plt.savefig("evidence/screenshots/session4_baselines.png", dpi=150)

    results = pd.DataFrame(rows)
    results.to_csv(f"{RESULTS_DIR}/baseline_results_seed{SEED}.csv", index=False)
    pd.set_option("display.width", 120)
    print(results.round(3).to_string(index=False))
    print("Saved table to experiments/results/baseline_results_seed1.csv")
    print("Saved plot to evidence/screenshots/session4_baselines.png")


if __name__ == "__main__":
    main()