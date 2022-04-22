from launch_ros.substitutions import FindPackageShare
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare("inference_helper_sample_ros"),
                    "launch",
                    "wrapper_generate_launch_description.launch.py"
                ])
            ]),
            launch_arguments={
                "name_exe": "seg_paddleseg_cityscapessota_exe",
            }.items()
        )
    ])
