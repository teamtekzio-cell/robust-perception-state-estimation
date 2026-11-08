"""
generate_sensor_data.py - Create noisy sensor data for every trajectory (Session 3).

Reads  : experiments/results/ground_truth_<name>.csv   (from Session 2)
Writes : experiments/results/sensors_<name>_seed<N>.csv
Plot   : evidence/screenshots/session3_sensor_readings.png

Run from the project root:
    python src/generate_sensor_data.py
"""
import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

from sensors import SensorConfig, simulate_sensors

TRAJECTORIES = ["square", "circle", "figure8"]
SEED = 1
RESULTS_DIR = "experiments/results"
PLOT_PATH = "evidence/screenshots/session3_sensor_readings.png"


def main():
    config = SensorConfig()
    print("Sensor settings:", config)

    for name in TRAJECTORIES:
        gt = pd.read_csv(f"{RESULTS_DIR}/ground_truth_{name}.csv")
        sensors = simulate_sensors(gt, config, seed=SEED)
        out_path = f"{RESULTS_DIR}/sensors_{name}_seed{SEED}.csv"
        sensors.to_csv(out_path, index=False)

        # Check: measured minus true should show the noise and bias we put in
        gyro_err = sensors.meas_gyro - gt.omega
        left_err = sensors.meas_v_left - gt.v_left
        print(f"{name:8s}: {len(sensors)} readings -> {out_path}")
        print(f"          left wheel error: mean {left_err.mean():+.4f}, std {left_err.std():.4f} m/s")
        print(f"          gyro error      : mean {gyro_err.mean():+.4f}, std {gyro_err.std():.4f} rad/s")

    # Plot the first 20 s of the square trajectory: true vs measured
    gt = pd.read_csv(f"{RESULTS_DIR}/ground_truth_square.csv")
    sensors = pd.read_csv(f"{RESULTS_DIR}/sensors_square_seed{SEED}.csv")
    window = gt.t <= 20

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(10, 6), sharex=True)
    ax1.plot(sensors.t[window], sensors.meas_v_right[window], ".", ms=2, alpha=0.5, label="measured (encoder)")
    ax1.plot(gt.t[window], gt.v_right[window], "r", lw=2, label="true")
    ax1.set_ylabel("right wheel speed (m/s)")
    ax1.set_title("Session 3: simulated sensor readings vs truth (square, first 20 s)")
    ax1.legend(loc="upper right"); ax1.grid(alpha=0.3)

    ax2.plot(sensors.t[window], sensors.meas_gyro[window], ".", ms=2, alpha=0.5, label="measured (gyroscope)")
    ax2.plot(gt.t[window], gt.omega[window], "r", lw=2, label="true")
    ax2.set_ylabel("turning rate (rad/s)")
    ax2.set_xlabel("time (s)")
    ax2.legend(loc="upper right"); ax2.grid(alpha=0.3)

    plt.tight_layout()
    plt.savefig(PLOT_PATH, dpi=150)
    print(f"Saved plot to {PLOT_PATH}")


if __name__ == "__main__":
    main()