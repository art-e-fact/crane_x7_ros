import os
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

import numpy as np
import matplotlib.pyplot as plt
import rclpy

from geometry_msgs.msg import PoseArray
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
        launch_arguments={"use_sim_time": "true", "example": "cartesian_path"}.items(),
    )


    crane_x7_pose_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/world/default/dynamic_pose/info@geometry_msgs/msg/PoseArray@ignition.msgs.Pose_V"
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            sim,
            crane_x7_pose_bridge,
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

    def get_ee_pose_index(self):
        # Command to run in the background
        command = "ign topic -t /world/default/dynamic_pose/info -e"

        # Start the subprocess and redirect stdout and stderr to pipes
        process = subprocess.Popen(
            command,
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            universal_newlines=True,
        )

        # Initialize flags to track whether the target parts have been found
        found_pose = False
        found_name = False

        # Initialize the variable to store the output
        output_call = ""

        # Initialize a counter for the word "pose"
        pose_count = 0

        while True:
            # Read a line from the stdout (non-blocking)
            output_line = process.stdout.readline()

            # Check if the process has finished
            if process.poll() is not None and not output_line:
                break

            if output_line:
                # Append the output_line to the variable
                output_call += output_line

                # Count the number of times "pose" appears in the output_line
                pose_count += output_line.count("pose")

                # Check if 'pose {' is in the output_line
                if "pose {" in output_line:
                    found_pose = True

                # Check if 'name: "crane_x7_gripper_base_link"' is in the output_line
                if 'name: "crane_x7_gripper_base_link"' in output_line:
                    found_name = True

                # Check if both parts have been found
                if found_pose and found_name:
                    break

        # Close the subprocess
        process.kill()

        pose_count = pose_count - 1

        return pose_count

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

        return position_x_array, position_y_array

    def plot(self, x_values_expected, y_values_expected, x_values_sim, y_values_sim):
        # Plot the points
        plt.plot(x_values_expected, y_values_expected, label="Expected Path")
        plt.plot(x_values_sim, y_values_sim, label="Sim Path")

        plt.xlabel("X Position")
        plt.ylabel("Y Position")
        plt.title("Sim Vs Expected Cartesian Path Trajectory")
        plt.legend()

        plt.grid(True)
        plt.show()

    def get_ee_pose(self, gripper_position):
        self.gripper_x_pose_list.append(gripper_position.x)
        self.gripper_y_pose_list.append(gripper_position.y)

    def test_joint_positions(self, proc_output):
        """
        Test case to see if box has moved, if the values are no longer equal it means that the block has shifted position
        """

        self.gripper_x_pose_list = []
        self.gripper_y_pose_list = []

        x_values_expected, y_values_expected = self.crane_x7_cartesian_path()

        proc_output.assertWaitFor("Goal request accepted!", timeout=180)

        pose_index = self.get_ee_pose_index()

        proc_output.assertWaitFor("Cartesian Path Beginning", timeout=180)

        sub = self.node.create_subscription(
            PoseArray,
            "/world/default/dynamic_pose/info",
            lambda msg: self.get_ee_pose(msg.poses[pose_index].position),
            10,
        )

        rclpy.spin_once(self.node, timeout_sec=0.1)

        try:

            # while True:
            #     rclpy.spin_once(self.node, timeout_sec=0.1)

            #     if proc_output.assertWaitFor("Cartesian Path Beginning", timeout=180):
            #         break

            end_time = time.time() + 60  # Need to fix time

            while time.time() < end_time:
                rclpy.spin_once(self.node, timeout_sec=0.1)
                # Need to fix the amount of data saved

        finally:
            self.node.destroy_subscription(sub)

        x_values_sim = np.array(self.gripper_x_pose_list)
        y_values_sim = np.array(self.gripper_y_pose_list)

        self.plot(x_values_expected, y_values_expected, x_values_sim, y_values_sim)
