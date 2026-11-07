"""Session 1: confirm the Python environment works and produce a first plot."""
import sys
import numpy as np
import scipy
import matplotlib
matplotlib.use("Agg")  # save plots to files instead of opening windows
import matplotlib.pyplot as plt

print("Python    :", sys.version.split()[0])
print("NumPy     :", np.__version__)
print("SciPy     :", scipy.__version__)
print("Matplotlib:", matplotlib.__version__)

# A noisy measurement around a true value: a preview of what sensors do
rng = np.random.default_rng(seed=42)        # fixed seed = repeatable result
true_value = 5.0
measurements = true_value + rng.normal(0, 0.5, size=200)

plt.figure(figsize=(7, 3.5))
plt.plot(measurements, ".", alpha=0.6, label="noisy sensor readings")
plt.axhline(true_value, color="red", label="true value")
plt.title("Session 1 check: noisy measurements around a true value")
plt.xlabel("time step"); plt.ylabel("value"); plt.legend()
plt.tight_layout()
plt.savefig("evidence/screenshots/session1_noise_check.png", dpi=150)
print("Mean of measurements:", round(measurements.mean(), 3), "(true value is 5.0)")
print("Saved plot to evidence/screenshots/session1_noise_check.png")