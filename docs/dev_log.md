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