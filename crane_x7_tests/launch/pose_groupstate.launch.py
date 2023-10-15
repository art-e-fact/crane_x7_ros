import os
import sys
from datetime import datetime
from time import sleep

import pytest
import unittest
import launch_testing
from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution
import launch_testing.markers

import subprocess

from launch_testing.asserts import assertSequentialStdout

from sensor_msgs.msg import JointState

# from std_msgs.msg import String


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

    joint_values = Node(
        package="crane_x7_tests",
        executable="joint_pose",
    )

    return LaunchDescription(
        [
            sim,
            TimerAction(period=10.0, actions=[demo]),
            joint_values,
            launch_testing.actions.ReadyToTest(),
        ]
    ), {"demo": demo, "sim": sim}


class TestPositionCheck(unittest.TestCase):
    def test_joint_positions(self, proc_output):
        """
        Test case to see if box has moved, if the values are no longer equal it means that the block has shifted position
        """

        proc_output.assertWaitFor(
            'Joint_Position: "[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]',
            timeout=180,
        )
        proc_output.assertWaitFor(
            'Joint_Position: "[0.0, 0.0, -2.2, 0.06, 0.4, 0.48, 0.0, 0.0, 0.0]',
            timeout=180,
        )
