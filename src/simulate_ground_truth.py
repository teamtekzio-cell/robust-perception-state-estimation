"""
simulate_ground_truth.py - Drive the robot along each trajectory and save
its TRUE path (ground truth). Later sessions compare every estimate
against these files.

Run from the project root:
    python src/simulate_ground_truth.py
"""
import os
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from robot_model import DiffDriveRobot
from trajectories import TRAJECTORIES

DT = 0.02  # time step in seconds (50 updates per second)
RESULTS_DIR = "experiments/results"
PLOT_PATH = "evidence/screenshots/session2_ground_truth.png"


def simulate(segments, dt=DT):
    robot = DiffDriveRobot()
    rows = []
    t = 0.0
    rows.append(dict(t=t, x=robot.x, y=robot.y, theta=robot.theta,
                     v=0.0, omega=0.0, v_left=0.0, v_right=0.0))
    for v_cmd, omega_cmd, duration in segments:
        steps = round(duration / dt)
        v_left, v_right = robot.inverse_kinematics(v_cmd, omega_cmd)
        for _ in range(steps):
            v, omega = robot.step(v_left, v_right, dt)
            t += dt
            rows.append(dict(t=round(t, 4), x=robot.x, y=robot.y, theta=robot.theta,
                             v=v, omega=omega, v_left=v_left, v_right=v_right))
    return pd.DataFrame(rows)


def main():
    os.makedirs(RESULTS_DIR, exist_ok=True)
    os.makedirs(os.path.dirname(PLOT_PATH), exist_ok=True)

    fig, axes = plt.subplots(1, 3, figsize=(13, 4.5))
    for ax, (name, make_segments) in zip(axes, TRAJECTORIES.items()):
        df = simulate(make_segments())
        csv_path = f"{RESULTS_DIR}/ground_truth_{name}.csv"
        df.to_csv(csv_path, index=False)

        end = df.iloc[-1]
        print(f"{name:8s}: {len(df):5d} samples, {end.t:6.1f} s, "
              f"end pose x={end.x:+.3f} m, y={end.y:+.3f} m, theta={end.theta:+.3f} rad "
              f"-> {csv_path}")

        ax.plot(df.x, df.y, linewidth=2)
        ax.plot(df.x.iloc[0], df.y.iloc[0], "go", label="start")
        ax.plot(end.x, end.y, "rx", markersize=10, label="end")
        ax.set_title(f"Ground truth: {name}")
        ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
        ax.set_aspect("equal"); ax.grid(True, alpha=0.3); ax.legend(fontsize=8, loc="upper right")

    plt.tight_layout()
    plt.savefig(PLOT_PATH, dpi=150)
    print(f"Saved plot to {PLOT_PATH}")


if __name__ == "__main__":
    main()