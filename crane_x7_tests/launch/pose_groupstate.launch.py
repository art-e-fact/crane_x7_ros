import os
import sys
from datetime import datetime
from time import sleep

import pytest
import unittest
import launch_testing
from launch import LaunchDescription
from launch.actions import TimerAction

from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import launch_testing.markers
import rclpy

from sensor_msgs.msg import JointState

import time


@pytest.mark.launch_test
@launch_testing.markers.keep_alive
def generate_test_description():
    proc_env = os.environ.copy()
    proc_env["PYTHONUNBUFFERED"] = "1"

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("crane_x7_gazebo"),
                        "launch",
                        "crane_x7_with_table.launch.py",
                    ]
                )
            ]
        ),
    )

    demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [
                PathJoinSubstitution(
                    [
                        FindPackageShare("crane_x7_examples"),
                        "launch",
                        "example.launch.py",
                    ]
                )
            ]
        ),
        launch_arguments={"use_sim_time": "true", "example": "pose_groupstate"}.items(),
    )

    return LaunchDescription(
        [
            sim,
            TimerAction(period=10.0, actions=[demo]),
            launch_testing.actions.ReadyToTest(),
        ]
    ), {"demo": demo, "sim": sim}


class TestPositionCheck(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = rclpy.create_node("test_node")

    def get_joints(self, joint_position):
        """
        callback storing joint state positions
        """
        self.rounded_joint_values = []
        decimal_place = 2
        for i in range(len(joint_position)):
            rounded_joint = round(joint_position[i], decimal_place)

            if rounded_joint == -0.0:
                rounded_joint = 0.0

            self.rounded_joint_values.append(rounded_joint)

    def test_joint_positions(self, proc_output):
        """
        Test case to see if the final joint groupstate is in the home position
        """

        joint_position_home = [0.0, 0.0, -2.2, 0.06, 0.4, 0.48, 0.0, 0.0, 0.0]

        sub = self.node.create_subscription(
            JointState,
            "/joint_states",
            lambda msg: self.get_joints(msg.position),
            10,
        )

        rclpy.spin_once(self.node, timeout_sec=0.1)

        try:
            end_time = time.time() + 30  # Need to fix time

            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=0.1)

        finally:
            self.node.destroy_subscription(sub)

        decimal = 2

        self.assertAlmostEqual(
            self.rounded_joint_values[2], joint_position_home[2], decimal
        )
