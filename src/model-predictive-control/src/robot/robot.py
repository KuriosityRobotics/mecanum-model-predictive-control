import rospy
from geometry_msgs.msg import *
from nav_msgs.msg import Odometry
from std_msgs.msg import Header
from tf.transformations import quaternion_from_euler
from typing import Literal

import numpy as np

from physics.dynamics import robot_acceleration


class Robot:
    def __init__(self):
        self.motor_voltages = np.zeros(4)
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)

    def tick(self, dt: float):
        acceleration = robot_acceleration(self.position, self.velocity, self.motor_voltages).reshape((3,))
        self.position += self.velocity * dt
        self.velocity += acceleration * dt

    def set_motor(self, motor: Literal[1, 2, 3, 4], voltage: float):
        if not(-12 <= voltage <= 12):
            raise ValueError(f"Invalid voltage:  {voltage:.2f} V")

        self.motor_voltages[motor - 1] = voltage

    def get_odometry(self) -> Odometry:
        [x, y, theta] = self.position
        pose = Pose(Point(x, y, 0), Quaternion(*quaternion_from_euler(0, 0, theta)))

        [vx, vy, psi] = self.velocity
        twist = Twist(Vector3(vx, vy, 0), Vector3(0, 0, psi))

        return Odometry(Header(None, None, "map"), "map", PoseWithCovariance(pose, [0.] * 36), TwistWithCovariance(twist, [0.] * 36))
