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

# import ros2bag2video
import subprocess


@pytest.mark.launch_test
def generate_test_description():
    proc_env = os.environ.copy()
    proc_env["PYTHONUNBUFFERED"] = "1"
    # TODO switch to package
    yyyymmddhhmmss = datetime.now().strftime("%Y_%m_%d-%H_%M_%S")
    rosbag_filepath = "rosbag2_" + yyyymmddhhmmss
    rosbag_cmd = [
        "ros2",
        "bag",
        "record",
        "/observation_camera/image",
        "-o",
        rosbag_filepath,
    ]
    spawn_cam = Node(
     package='ros_ign_gazebo',
     executable='create',
     output='screen',
     arguments=["-file", "./cam.sdf"]
    )
    bag_recorder = ExecuteProcess(cmd=rosbag_cmd, output="screen", env=proc_env)
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            ["crane_x7_gazebo", "/launch/crane_x7_with_table.launch.py"]
        )
    )
    controller = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            ["crane_x7_examples", "/launch/example.launch.py"]
        ),
        launch_arguments={"use_sim_time": "true", "example": "gripper_control"}.items(),
    )
    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        arguments=[
            "/observation_camera/image@sensor_msgs/msg/Image@gz.msgs.Image",
        ],
        output="screen",
    )
    return LaunchDescription(
        [
            sim,
            TimerAction(period=5.0, actions=[spawn_cam]),
            TimerAction(period=10.0, actions=[controller]),
            bridge,
            TimerAction(period=8.0, actions=[bag_recorder]),
            launch_testing.actions.ReadyToTest(),
        ]
    ), {"controller": controller, "rosbag_filepath": rosbag_filepath}


class TestGripper(unittest.TestCase):
    def test_plan_execution(self, proc_output, controller):
        # This will match stdout from test_process.

        # [gripper_control-10] [INFO] [1692868718.520351309] [move_group_interface]: Plan and Execute request complete!
        # [INFO] [gripper_control-10]: process has finished cleanly [pid 21220]
        # with launch_testing.asserts.assertSequentialStdout(proc_output, "gripper_control-10") as cm:
        #    cm.assertInStdout("Plan and Execute request complete!")
        #    cm.assertInStdout("Plan and Execute request complete!")
        #    cm.assertInStdout("Loop 2")
        proc_output.assertWaitFor("Plan and Execute request complete!", timeout=180)
        sleep(5)


@launch_testing.post_shutdown_test()
class TestAfterShutdown(unittest.TestCase):
    def test_exit(self, proc_info, rosbag_filepath):
        print(rosbag_filepath)
        output_video_name = "output/output_video.mp4"
        args = [
            "./ros2bag2video.py",
            "--topic",
            "/observation_camera/image",
            "--fps",
            "20",
            "--rate",
            "1.0",
            "-o",
            output_video_name,
            rosbag_filepath,
        ]
        args = [
            "python",
            "./ros2bag2video.py",
            "--topic",
            "/observation_camera/image",
            "--fps",
            "20",
            "--rate",
            "1.0",
            "-o",
            output_video_name,
            rosbag_filepath,
        ]
        #'--fps', '2',
        try:
            subprocess.run(["mkdir", "output"])
        except Exception as e:
            print(e)
        try:
            subprocess.run(["rm", "output/*.mp4"])
            subprocess.run(["rm", "output/*.webm"])
            subprocess.run(args)
            subprocess.run(
                [
                    "ffmpeg",
                    "-i",
                    output_video_name,
                    "-c:v",
                    "libvpx-vp9",
                    "-crf",
                    "30",
                    "-b:v",
                    "0",
                    "-y",
                    output_video_name.split(".")[0] + ".webm",
                ]
            )
        except Exception as e:
            print("error")
            print(e)

        # stop_ignition_cmd = ExecuteProcess(
        #    cmd=['pkill', 'ign'],
        #    output='screen'
        # )
        # stop_ignition_cmd.execute()
        subprocess.run(["pkill", "ign"])
