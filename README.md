```sh
xhost local:    # if needed
docker run -it --name foxytest1 -e DISPLAY=$DISPLAY osrf/ros:foxy-desktop
# docker run -it --name ros_foxy_1 -e DISPLAY=$DISPLAY --net=host -v `pwd`:/InferenceHelper_Sample_ROS osrf/ros:foxy-desktop
# docker run -it --name ros_foxy_1 -e DISPLAY=$DISPLAY --net=host -v /dev:/dev -v `pwd`:/InferenceHelper_Sample_ROS --privileged osrf/ros:foxy-desktop
# docker start ros_foxy_1
# docker exec -it ros_foxy_1 /ros_entrypoint.sh bash

# export DISPLAY=192.168.1.2:0
```

```
apt update && apt install -y libopencv-dev ros-foxy-image-pipeline unzip

cd src/InferenceHelper_Sample_ROS
git submodule update --init
sh inference_helper_sample_ros/src/inference_helper/third_party/download_prebuilt_libraries.sh
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





colcon build  --cmake-args -DINFERENCE_HELPER_ENABLE_TFLITE_DELEGATE_XNNPACK=OFF -DINFERENCE_HELPER_ENABLE_ONNX_RUNTIME=ON
colcon build  --cmake-args -DINFERENCE_HELPER_ENABLE_TFLITE_DELEGATE_XNNPACK=ON -DINFERENCE_HELPER_ENABLE_ONNX_RUNTIME=OFF

export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:build/inference_helper_sample_ros/
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:src/InferenceHelper_Sample_ROS/inference_helper_sample_ros/src/inference_helper//third_party/onnxruntime_prebuilt/linux-x64/lib/

ros2 launch inference_helper_sample_ros depth_lapdepth.launch.py image_filename:=/root/dev_ws/src/InferenceHelper_Sample_ROS/resource/dashcam_01.jpg
ros2 launch inference_helper_sample_ros all.launch.py image_filename:=/root/dev_ws/src/InferenceHelper_Sample_ROS/resource/dashcam_01.jpg

ros2 run image_publisher image_publisher_node /justdoit.mp4
ros2 run inference_helper_sample_ros cls_mobilenet_v2_exe

