![demo_adas](00_doc/demo_adas.jpg)

# InferenceHelper_Sample_ROS
- DNN node collection using Inference Helper (https://github.com/iwatake2222/InferenceHelper ) in ROS2

![class_diagram](00_doc/class_diagram.png)

## Node Collection
| Node | Tested frameworks | Description |
|------|-------------------| ----------- |
| transport | None | Just transport input image |
| cls_mobilenet_v2 | <ul><li>INFERENCE_HELPER_ENABLE_TFLITE_DELEGATE_XNNPACK</li><li>INFERENCE_HELPER_ENABLE_ONNX_RUNTIME</li><li>INFERENCE_HELPER_ENABLE_MNN</li><li>INFERENCE_HELPER_ENABLE_NCNN</li><li>INFERENCE_HELPER_ENABLE_OPENCV</li></ul> | Classification by MobileNetV2 |
| det_yolox | <ul><li>INFERENCE_HELPER_ENABLE_TFLITE_DELEGATE_XNNPACK</li><li>INFERENCE_HELPER_ENABLE_ONNX_RUNTIME</li><li>INFERENCE_HELPER_ENABLE_MNN</li><li>INFERENCE_HELPER_ENABLE_NCNN</li></ul> | Detection by YOLOX-Nano |
| seg_paddleseg_cityscapessota | <ul><li>INFERENCE_HELPER_ENABLE_TFLITE_DELEGATE_XNNPACK</li><li>INFERENCE_HELPER_ENABLE_ONNX_RUNTIME</li></ul> | Segmentation by PaddleSeg CityScapesSOTA |
| depth_lapdepth | <ul><li>INFERENCE_HELPER_ENABLE_TFLITE_DELEGATE_XNNPACK</li><li>INFERENCE_HELPER_ENABLE_ONNX_RUNTIME</li></ul> | Depth by LapDepth<br> Point Cloud |

- There are lots of projects which haven't been ported. You can add more by referring the following repos
    - https://github.com/iwatake2222/InferenceHelper_Sample
    - https://github.com/iwatake2222/play_with_tflite
    - https://github.com/iwatake2222/play_with_tensorrt
    - https://github.com/iwatake2222/play_with_ncnn
    - https://github.com/iwatake2222/play_with_mnn
- `Tested frameworks` above are just frameworks I tested. You can try another framework

## Tested Environment
- Host PC
    - Ubuntu 20.04
    - Ubuntu 20.04 + Docker
    - Windows 11 + WSL2 (Ubuntu 20.04)
- ROS2 version
    - Foxy

## How to Run
### Create docker container for ROS2
- This is just an example. You can skip this if you have your ROS2 environment in you PC

```sh:in_your_host_PC
xhost local:    # if needed
docker run -it --name ros2_foxy_1 -e DISPLAY=$DISPLAY --net=host osrf/ros:foxy-desktop
# docker start ros2_foxy_1
# docker exec -it ros2_foxy_1 /ros_entrypoint.sh bash
```

### Install software

```sh:in_container
# export DISPLAY=192.168.1.2:0   # if needed
apt update && apt install -y libopencv-dev ros-foxy-image-pipeline unzip
```

### Create ROS2 workspace and download this repo
```sh:in_container
source /opt/ros/foxy/setup.bash
mkdir -p ~/dev_ws/src
cd ~/dev_ws/src

git clone https://github.com/iwatake2222/InferenceHelper_Sample_ROS.git
cd InferenceHelper_Sample_ROS
git submodule update --init
sh inference_helper_sample_ros/src/inference_helper/third_party/download_prebuilt_libraries.sh
sh download_resource.sh
```

### Build
```sh:in_container
cd ~/dev_ws
rosdep install -i --from-path src --rosdistro foxy -y
colcon build  --cmake-args -DINFERENCE_HELPER_ENABLE_TFLITE_DELEGATE_XNNPACK=OFF -DINFERENCE_HELPER_ENABLE_ONNX_RUNTIME=ON
# colcon build  --cmake-args -DINFERENCE_HELPER_ENABLE_TFLITE_DELEGATE_XNNPACK=ON -DINFERENCE_HELPER_ENABLE_ONNX_RUNTIME=OFF
```

- Enable one of `INFERENCE_HELPER_ENABLE_***` options when you build.
- Above commands are just an example. Refer to https://github.com/iwatake2222/InferenceHelper#cmake-options to find more build options
- Note
    - Some frameworks may not be supported in your environment
    - Some models may not be supported in your environment / framework you selected


### Set library path
```sh:in_container
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:build/inference_helper_sample_ros/
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:src/InferenceHelper_Sample_ROS/inference_helper_sample_ros/src/inference_helper/third_party/onnxruntime_prebuilt/linux-x64/lib/
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:src/InferenceHelper_Sample_ROS/inference_helper_sample_ros/src/inference_helper/third_party/mnn_prebuilt/ubuntu/lib/
```

- You need to specify framework library path in the same terminal as you run the node later
- There may be a way to avoid it. HELP WANTED!!

### Run
- Run manually
    - You need 3 terminals

```sh
cd ~/dev_ws
. install/setup.bash

ros2 run image_publisher image_publisher_node src/InferenceHelper_Sample_ROS/resource/dog.jpg
ros2 run inference_helper_sample_ros det_yolox_exe
ros2 run image_view image_view --ros-args --remap image:=/transported_image_raw
```

- Run launch file

```sh
cd ~/dev_ws
. install/setup.bash

ros2 launch inference_helper_sample_ros cls_mobilenet_v2.launch.py image_filename:=src/InferenceHelper_Sample_ROS/resource/parrot.jpg
ros2 launch inference_helper_sample_ros det_yolox.launch.py
ros2 launch inference_helper_sample_ros depth_lapdepth.launch.py
ros2 launch inference_helper_sample_ros seg_paddleseg_cityscapessota.launch.py image_filename:=src/InferenceHelper_Sample_ROS/resource/dashcam_02.jpg
```

- Run ADAS demo

```sh
cd ~/dev_ws
. install/setup.bash

ros2 launch inference_helper_sample_ros adas.launch.py
```

## Node Specs
![graph](00_doc/rqt_graph_0.png)

- Subscribed Topics
    - `/image_raw` (`sensor_msgs/msg/Image` )
- Published Topics
    - `/transported_image_raw` (`sensor_msgs/msg/Image` )
    - `/inference_result`
        - inference_helper_sample_ros_interface/msg/Classification.msg, BoundingBoxList.msg, etc.
        - `sensor_msgs/msg/PointCloud2` (depth)
- Parameters
    - `topic_image_sub`
        - `/image_raw`
    - `topic_image_pub`
        - `/transported_image_raw`
    - `topic_result_pub`
        - `/inference_result`
    - `work_dir`
        - `/root/dev_ws/src/InferenceHelper_Sample_ROS/resource/`
    - `thread_num`
        - 4


# License
- Copyright 2022 iwatake2222
- Licensed under the Apache License, Version 2.0

# Acknowledgements
- This project utilizes OSS (Open Source Software)
    - [NOTICE.md](NOTICE.md)

