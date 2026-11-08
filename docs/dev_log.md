# Development Log

## 2026-11-07 – Session 1: Project setup
- Created private GitHub repo and project folder structure
- Set up Python 3.14 virtual environment (numpy, scipy, matplotlib, pandas, pytest)
- Saved exact library versions in requirements.txt
- Ran src/check_setup.py successfully; saved first plot (noisy measurements vs true value)
- Next: build 2D differential-drive robot simulator


## 2026-11-08 – Session 2: Robot simulator and ground truth
- Implemented differential-drive kinematics (forward, inverse, midpoint pose update)
- Created square, circle and figure-8 trajectories (straight, rotation, curved motion)
- Generated ground-truth CSVs and plot; 5 unit tests passing
- Next: simulate noisy sensors (wheel odometry, IMU)


## 2026-11-08 – Session 3: Sensor simulation
- Implemented wheel encoder model (Gaussian noise + 1% right-wheel scale error)
- Implemented gyroscope model (Gaussian noise + constant bias)
- Generated sensor CSVs for 3 trajectories (seed 1); measured errors match configured values
- 9 unit tests passing
- Next: dead-reckoning baselines (odometry-only, gyro-only heading)


## 2026-11-08 – Session 4: Dead-reckoning baselines and metrics
- Implemented odometry-only and odometry + gyro-heading dead reckoning
- Implemented metrics: position RMSE, heading RMSE, final and max position error
- Ran baselines on 3 trajectories (seed 1); gyro heading roughly halved position RMSE
- Observation: figure-8 final error is small while max error is large (drift partly cancels) -> final error alone is misleading
- 13 unit tests passing
- Next: Extended Kalman Filter fusing odometry and gyroscope


## 2026-11-08 – Session 5: Extended Kalman Filter
- Implemented EKF, state [x, y, theta, v, omega]; predict with motion model; updates from wheel odometry and gyro
- R matrices derived from sensor noise settings
- EKF vs baselines (seed 1): ~50% lower RMSE than odometry-only, but slightly worse than odometry + gyro heading
- Consistency check: heading error inside EKF's ±2σ only 1% of time -> EKF overconfident; systematic errors (gyro bias, wheel scale) not modelled
- 17 unit tests passing
- Next: experiments over multiple seeds and noise levels


## 2026-11-08 – Session 6: Repeated experiments (10 seeds)
- Exp A (nominal), Exp B (noise only, systematic errors off), Exp C (noise levels 0.5x–4x)
- Systematic errors cause most drift: removing them cut position RMSE ~10–20x
- EKF consistency: 1.5–4% inside ±2σ with systematic errors vs 90–98% with noise only -> confirms cause
- EKF matched but did not beat odom + gyro heading: gyro ~4x more precise for omega, so fusion adds little
- Error scales roughly linearly with sensor noise
- Config saved to experiments/configs/session6_config.json
- Next: sensor-failure test (temporary sensor outage)