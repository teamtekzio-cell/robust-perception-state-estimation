"""
robot_model.py - Differential-drive robot kinematics (Session 2).

The robot has two wheels on one axle. Its pose is (x, y, theta):
    x, y  : position in metres (world frame)
    theta : heading in radians (0 = facing +x, pi/2 = facing +y)

Default dimensions are close to a TurtleBot3 Burger, the robot we will
later use in Gazebo, so both simulations describe the same kind of robot.
"""
import math


def wrap_angle(angle):
    """Keep an angle between -pi and +pi (e.g. 370 deg becomes 10 deg)."""
    return math.atan2(math.sin(angle), math.cos(angle))


class DiffDriveRobot:
    def __init__(self, wheel_base=0.160, x=0.0, y=0.0, theta=0.0):
        self.wheel_base = wheel_base  # L: distance between the wheels (m)
        self.x = x
        self.y = y
        self.theta = theta

    # ---- Forward kinematics: wheel speeds -> robot motion ----
    def forward_kinematics(self, v_left, v_right):
        v = (v_right + v_left) / 2.0                    # forward speed (m/s)
        omega = (v_right - v_left) / self.wheel_base    # turning rate (rad/s)
        return v, omega

    # ---- Inverse kinematics: desired robot motion -> wheel speeds ----
    def inverse_kinematics(self, v, omega):
        v_left = v - omega * self.wheel_base / 2.0
        v_right = v + omega * self.wheel_base / 2.0
        return v_left, v_right

    # ---- Pose update: move the robot forward by one small time step ----
    def step(self, v_left, v_right, dt):
        v, omega = self.forward_kinematics(v_left, v_right)
        # Midpoint heading: use the heading halfway through the step.
        # This is more accurate than plain Euler (using the old heading).
        theta_mid = self.theta + omega * dt / 2.0
        self.x += v * math.cos(theta_mid) * dt
        self.y += v * math.sin(theta_mid) * dt
        self.theta = wrap_angle(self.theta + omega * dt)
        return v, omega

    def pose(self):
        return (self.x, self.y, self.theta)