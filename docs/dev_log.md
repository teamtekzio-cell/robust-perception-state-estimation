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