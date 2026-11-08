"""
test_robot_model.py - Automated checks that the kinematics behave as theory says.
Run from the project root:  pytest -v
"""
import math
import pytest
from robot_model import DiffDriveRobot, wrap_angle


def test_equal_wheel_speeds_drive_straight():
    robot = DiffDriveRobot()
    for _ in range(100):                      # 100 steps x 0.01 s = 1 s
        robot.step(0.2, 0.2, 0.01)
    assert robot.x == pytest.approx(0.2)      # 0.2 m/s for 1 s = 0.2 m
    assert robot.y == pytest.approx(0.0)
    assert robot.theta == pytest.approx(0.0)


def test_opposite_wheel_speeds_rotate_in_place():
    robot = DiffDriveRobot()
    for _ in range(100):
        robot.step(-0.1, 0.1, 0.01)
    assert robot.x == pytest.approx(0.0)      # position does not change
    assert robot.y == pytest.approx(0.0)
    assert robot.theta > 0                    # right wheel forward -> turns left


def test_inverse_then_forward_kinematics_round_trip():
    robot = DiffDriveRobot()
    v_l, v_r = robot.inverse_kinematics(0.15, 0.7)
    v, omega = robot.forward_kinematics(v_l, v_r)
    assert v == pytest.approx(0.15)
    assert omega == pytest.approx(0.7)


def test_full_circle_returns_to_start():
    robot = DiffDriveRobot()
    v, radius, dt = 0.2, 1.0, 0.02
    v_l, v_r = robot.inverse_kinematics(v, v / radius)
    steps = round(2 * math.pi * radius / v / dt)
    for _ in range(steps):
        robot.step(v_l, v_r, dt)
    assert robot.x == pytest.approx(0.0, abs=0.01)
    assert robot.y == pytest.approx(0.0, abs=0.01)


def test_wrap_angle_keeps_heading_in_range():
    assert wrap_angle(2 * math.pi + 0.1) == pytest.approx(0.1)
    assert wrap_angle(-math.pi - 0.1) == pytest.approx(math.pi - 0.1)