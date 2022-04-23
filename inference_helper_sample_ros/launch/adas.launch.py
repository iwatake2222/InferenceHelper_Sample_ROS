import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    ### Get env
    ws_dir = os.getcwd()

    rviz_config = os.path.join(
        get_package_share_directory("inference_helper_sample_ros"),
        "rviz",
        "adas_rviz.rviz"
    )

    ### Configuration
    image_filename = LaunchConfiguration("image_filename")
    work_dir = LaunchConfiguration("work_dir")
    thread_num = LaunchConfiguration("thread_num")

    ### Argument
    launch_args = [
        DeclareLaunchArgument(
            "image_filename",
            default_value=ws_dir + "/src/InferenceHelper_Sample_ROS/resource/" + "/dashcam_01.jpg"
        ),
        DeclareLaunchArgument(
            "work_dir",
            default_value=ws_dir + "/src/InferenceHelper_Sample_ROS/resource/"
        ),
        DeclareLaunchArgument(
            "thread_num",
            default_value="4"
        ),
    ]

    ### Execution
    exes = [
        ExecuteProcess(
            cmd=[[
                "ros2 run image_publisher image_publisher_node ",
                image_filename,
                " --ros-args --remap /image_raw:=",
                "/image_raw_adas"
            ]],
            shell=True
        ),
        # ExecuteProcess(
        #     cmd=[[
        #         "ros2 run image_view image_view --ros-args --remap image:=",
        #         "/transported_image_raw_det_yolox",
        #     ]],
        #     shell=True
        # ),
    ]

    ### Node
    nodes = [
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2_adas",
            arguments=['-d', rviz_config]
        ),
        Node(
            package="inference_helper_sample_ros",
            executable="det_yolox_exe",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "work_dir": work_dir,
                "thread_num": thread_num,
                "topic_image_sub": "/image_raw_adas",
                "topic_image_pub": "/transported_image_raw_det",
                "topic_result_pub": "result_det",
            }]
        ),
        Node(
            package="inference_helper_sample_ros",
            executable="seg_paddleseg_cityscapessota_exe",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "work_dir": work_dir,
                "thread_num": thread_num,
                "topic_image_sub": "/image_raw_adas",
                "topic_image_pub": "/transported_image_raw_seg",
                "topic_result_pub": "result_seg",
            }]
        ),
        Node(
            package="inference_helper_sample_ros",
            executable="depth_lapdepth_exe",
            output="screen",
            emulate_tty=True,
            parameters=[{
                "work_dir": work_dir,
                "thread_num": thread_num,
                "topic_image_sub": "/image_raw_adas",
                "topic_image_pub": "/transported_image_raw_depth",
                "topic_result_pub": "result_depth",
            }]
        ),
    ]
    
    return LaunchDescription(launch_args + exes + nodes)
