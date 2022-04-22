import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node

def generate_launch_description():
    ws_dir = os.getcwd()

    run_image_publisher = LaunchConfiguration("run_image_publisher")
    image_filename = LaunchConfiguration("image_filename")
    topic_image_sub = LaunchConfiguration("topic_image_sub")
    topic_image_pub = LaunchConfiguration("topic_image_pub")

    launch_args = [
        DeclareLaunchArgument(
            "run_image_publisher",
            default_value="True"
        ),
        DeclareLaunchArgument(
            "image_filename",
            default_value=ws_dir + "/src/InferenceHelper_Sample_ROS/resource/" + "/dog.jpg"
        ),
        DeclareLaunchArgument(
            "topic_image_sub",
            default_value="/image_raw"
        ),
        DeclareLaunchArgument(
            "topic_image_pub",
            default_value="/transported_image_raw"
        ),
    ]

    exes = [
        ExecuteProcess(
            condition=IfCondition(
                PythonExpression([
                    "'" + str(image_filename) + "'" " != ''",
                    " and ",
                    run_image_publisher,
                ])
            ),
            cmd=[[
                "ros2 run image_publisher image_publisher_node ",
                image_filename,
                " --ros-args --remap /image_raw:=",
                topic_image_sub
            ]],
            shell=True
        ),
    ]

    nodes = [
        Node(
            package="inference_helper_sample_ros",
            executable="transport_exe",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "topic_image_sub": topic_image_sub,
                "topic_image_pub": topic_image_pub,
            }]
        ),
    ]
    
    return LaunchDescription(launch_args + exes + nodes)
