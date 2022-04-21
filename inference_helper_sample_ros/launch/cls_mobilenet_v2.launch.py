from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

import os


def generate_launch_description():
    ws_dir = os.getcwd()
    launch_args = [
        DeclareLaunchArgument(
            "work_dir",
            default_value=ws_dir + "/src/InferenceHelper_Sample_ROS/resource/"
        ),
        DeclareLaunchArgument(
            "thread_num",
            default_value="4"
        ),
        DeclareLaunchArgument(
            "topic_image_sub",
            default_value="/image_raw"
        ),
        DeclareLaunchArgument(
            "topic_image_pub",
            default_value="/transported_image_raw"
        ),
        DeclareLaunchArgument(
            "topic_result_pub",
            default_value="/inference_result"
        ),
    ]
    nodes = [
        Node(
            package="inference_helper_sample_ros",
            executable="cls_mobilenet_v2_exe",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "work_dir": LaunchConfiguration("work_dir"),
                "thread_num": LaunchConfiguration("thread_num"),
                "topic_image_sub": LaunchConfiguration("topic_image_sub"),
                "topic_image_pub": LaunchConfiguration("topic_image_pub"),
                "topic_result_pub": LaunchConfiguration("topic_result_pub"),
            }]
        )
    ]
    return LaunchDescription(launch_args + nodes)
