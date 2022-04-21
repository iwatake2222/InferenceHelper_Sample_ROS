from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    launch_args = [
        DeclareLaunchArgument(
            "dummy",
            default_value="Default value by launch"
        ),
    ]
    nodes = [
        Node(
            package="inference_helper_sample_ros",
            executable="cls_mobilenet_v2_exe",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "dummy": LaunchConfiguration("dummy"),
            }]
        )
    ]
    return LaunchDescription(launch_args + nodes)
