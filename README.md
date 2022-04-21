```sh
xhost local:    # if needed
docker run -it --name foxytest1 -e DISPLAY=$DISPLAY osrf/ros:foxy-desktop
# docker run -it --name ros_foxy_1 -e DISPLAY=$DISPLAY --net=host -v `pwd`:/InferenceHelper_Sample_ROS osrf/ros:foxy-desktop
# docker run -it --name ros_foxy_1 -e DISPLAY=$DISPLAY --net=host -v /dev:/dev -v `pwd`:/InferenceHelper_Sample_ROS --privileged osrf/ros:foxy-desktop
# docker start ros_foxy_1
# docker exec -it ros_foxy_1 /ros_entrypoint.sh bash

# export DISPLAY=192.168.1.2:0
```

```sh
mkdir -p ~/dev_ws/src
cd ~/dev_ws/src

ln -s /InferenceHelper_Sample_ROS ./InferenceHelper_Sample_ROS
cd InferenceHelper_Sample_ROS
ros2 pkg create inference_helper_sample_ros_interface --build-type ament_cmake --dependencies rclcpp rclcpp_components std_msgs
ros2 pkg create inference_helper_sample_ros --build-type ament_cmake --dependencies rclcpp rclcpp_components std_msgs inference_helper_sample_ros_interface --node-name cls_mobilenet_v2

cd ../../
rosdep install -i --from-path src --rosdistro foxy -y
colcon build
```

