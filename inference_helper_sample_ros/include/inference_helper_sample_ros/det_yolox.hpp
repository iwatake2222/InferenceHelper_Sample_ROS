/* Copyright 2022 iwatake2222

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

    http://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
==============================================================================*/
#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "inference_helper_sample_ros/visibility_control.h"
#include "inference_helper_sample_ros_interface/msg/bounding_boxes.hpp"
#include "inference_helper_sample_ros_interface/msg/bounding_box.hpp"

#include "common_helper_cv.h"

class DetectionEngine;

namespace inference_helper_sample_ros
{

class DetYolox : public rclcpp::Node
{
public:
  INFERENCE_HELPER_SAMPLE_FOR_ROS_PUBLIC explicit DetYolox(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg);
  void read_parameter();

private:
  std::string prm_work_dir_;
  int32_t prm_thread_num_;
  std::string prm_topic_name_image_sub_;
  std::string prm_topic_name_image_pub_;
  std::string prm_topic_name_result_pub_;

  std::unique_ptr<image_transport::ImageTransport> it_;
  image_transport::Subscriber it_sub_;
  image_transport::Publisher it_pub_;

  rclcpp::Publisher<inference_helper_sample_ros_interface::msg::BoundingBoxes>::SharedPtr publisher_result_;

  std::unique_ptr<DetectionEngine> engine_;
  CommonHelper::NiceColorGenerator nice_color_generator_;

};

}
