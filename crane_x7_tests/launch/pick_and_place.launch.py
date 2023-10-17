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
import re


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
        launch_arguments={"use_sim_time": "true", "example": "pick_and_place"}.items(),
    )

    return LaunchDescription(
        [
            sim,
            TimerAction(
                period=15.0, actions=[demo]
            ),  # Need to optimize period, if under 10 seconds will not run
            launch_testing.actions.ReadyToTest(),
        ]
    ), {"demo": demo, "sim": sim}


def get_model_location(model_name):
    """
    Function to get the location of a model
    """

    command = ["ign", "model", "-m", model_name, "-p"]
    result = subprocess.run(command, check=True, text=True, stdout=subprocess.PIPE)
    result_output = result.stdout

    # Extract position data if it exists
    position_pattern = r"\[(.*?)\]"

    # Search for all matching instances of the list pattern in the text
    matches = re.findall(position_pattern, result_output)

    if len(matches) >= 2:
        position = matches[-2]

        # Extract the individual numbers from the matched string
        numbers = re.findall(r"[\d.-]+", position)

        if len(numbers) >= 3:
            x_variable = float(numbers[0])
            y_variable = float(numbers[1])
            z_variable = float(numbers[2])

            print("x, y, and z variables found in the text.")
        else:
            x_variable = 0.0
            y_variable = 0.0
            z_variable = 0.0
    else:
        print("No x, y, or z variables found in the text.")

    return x_variable, y_variable, z_variable


class TestPickAndPlace(unittest.TestCase):
    def test_box_moved(self, proc_output):
        """
        Test case to see if box has moved, if the values are no longer equal it means that the block has shifted position
        """

        decimal = 2
        model_name = "wood_cube_5cm"

        proc_output.assertWaitFor("Pick and Place Action done!", timeout=180)

        (
            x_actual_location,
            y_actual_location,
            z_actual_location,
        ) = get_model_location(model_name)

        x_desired_location = 0.2
        y_desired_location = 0.2
        z_desired_location = 1.015

        self.assertAlmostEqual(x_desired_location, x_actual_location, decimal)
        self.assertAlmostEqual(y_desired_location, y_actual_location, decimal)
        self.assertAlmostEqual(z_desired_location, z_actual_location, decimal)

        # sleep(5)
        # subprocess.run(["pkill", "ign"])


# @launch_testing.post_shutdown_test()
# class TestAfterShutdown(unittest.TestCase):
#     def test_exit():
#         subprocess.run(["pkill", "ign"])
