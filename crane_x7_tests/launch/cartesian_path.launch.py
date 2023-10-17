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
import numpy as np
import matplotlib.pyplot as plt



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
        launch_arguments={"use_sim_time": "true", "example": "cartesian_path"}.items(),
    )


    crane_x7_pose_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            '/world/default/dynamic_pose/info@geometry_msgs/msg/PoseArray@ignition.msgs.Pose_V'

        ],
        output="screen",
    )


    # joint_values = Node(
    #     package="crane_x7_tests",
    #     executable="joint_pose",
    # )

    return LaunchDescription(
        [
            sim,
            crane_x7_pose_bridge,
            TimerAction(period=10.0, actions=[demo]),
            launch_testing.actions.ReadyToTest(),
        ]
    ), {"demo": demo, "sim": sim}


class TestPositionCheck(unittest.TestCase):
    def crane_x7_cartesian_path(self):

        num_of_waypoints = 30
        repeat = 3
        radius = 0.1
        center_position_x = 0.3
        center_position_y = 0.0

        position_x_list = []
        position_y_list = []

        for j in range(repeat):
            for i in range(num_of_waypoints):
                theta = 2.0 * np.pi * (i / num_of_waypoints)
                position_x = center_position_x + radius * np.cos(theta)
                position_y = center_position_y + radius * np.sin(theta)

                position_x_list.append(position_x)
                position_y_list.append(position_y)

        # Convert the lists to NumPy arrays for easy plotting
        position_x_array = np.array(position_x_list)
        position_y_array = np.array(position_y_list)

        # Plot the points
        # plt.plot(position_x_array, position_y_array)
        # plt.xlabel('X Position')
        # plt.ylabel('Y Position')
        # plt.title('Waypoints')
        # plt.grid(True)
        # plt.show()

    def test_joint_positions(self, proc_output):
        """
        Test case to see if box has moved, if the values are no longer equal it means that the block has shifted position
        """

        proc_output.assertWaitFor(
        'Joint_Position: "[0.0, 0.0, -2.2, 0.06, 0.4, 0.48, 0.0, 0.0, 0.0]',
        timeout=180,
        )