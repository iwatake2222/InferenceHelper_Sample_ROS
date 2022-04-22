import os
from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration


def generate_include_launch_description(launch_name):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("inference_helper_sample_ros"),
                "launch",
                launch_name
            ])
        ]),
        launch_arguments={
            "run_image_publisher": "False",
            "topic_image_pub": "/transported_image_raw" + "_" + launch_name.split(".")[0],
            "topic_result_pub": "/inference_result" + "_" + launch_name.split(".")[0],
        }.items()
    )

def generate_launch_description():
    ws_dir = os.getcwd()
    resource_dir=ws_dir + "/src/InferenceHelper_Sample_ROS/resource/"

    image_filename = LaunchConfiguration("image_filename")

    launch_args = [
        DeclareLaunchArgument(
            "image_filename",
            default_value=resource_dir + "/dog.jpg",
        ),
    ]

    image_publisher = ExecuteProcess(
        cmd=[[
            "ros2 run image_publisher image_publisher_node ",
            image_filename
        ]],
        shell=True
    )

    sub_launches = [
        generate_include_launch_description("transport.launch.py"),
        generate_include_launch_description("cls_mobilenet_v2.launch.py"),
        generate_include_launch_description("det_yolox.launch.py"),
        generate_include_launch_description("seg_paddleseg_cityscapessota.launch.py"),
    ]

    return LaunchDescription(launch_args + [image_publisher] + sub_launches)

