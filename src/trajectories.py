"""
trajectories.py - Motion plans for the simulated robot (Session 2).

A trajectory is a list of segments: (forward speed v, turning rate omega, duration).
Three shapes cover the motion types required by Experiment 6:
    square  : straight lines + turns on the spot
    circle  : constant curve
    figure8 : a left curve followed by a right curve
"""
import math

MAX_V = 0.20  # m/s, just under the TurtleBot3 Burger limit (0.22 m/s)


def square(side=2.0, v=MAX_V, turn_rate=0.5):
    segments = []
    for _ in range(4):
        segments.append((v, 0.0, side / v))                         # drive straight
        segments.append((0.0, turn_rate, (math.pi / 2) / turn_rate))  # turn 90 deg on the spot
    return segments


def circle(radius=1.0, v=MAX_V):
    omega = v / radius                      # v = omega * r
    duration = 2 * math.pi / omega          # time for one full circle
    return [(v, omega, duration)]


def figure8(radius=1.0, v=MAX_V):
    omega = v / radius
    duration = 2 * math.pi / omega
    return [(v, omega, duration),           # left loop
            (v, -omega, duration)]          # right loop


TRAJECTORIES = {"square": square, "circle": circle, "figure8": figure8}