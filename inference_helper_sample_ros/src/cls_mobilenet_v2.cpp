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
#include <cstdlib>
#include <chrono>
#include <memory>

#include "opencv2/opencv.hpp"

#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"

#include "inference_helper_sample_ros/cls_mobilenet_v2.hpp"

#include "common_helper.h"
#include "common_helper_cv.h"
#include "engine/engine_cls_mobilenet_v2.hpp"

namespace inference_helper_sample_ros
{

static void DrawFps(cv::Mat& mat, double time_inference, cv::Point pos, double font_scale, int32_t thickness, cv::Scalar color_front, cv::Scalar color_back, bool is_text_on_rect = true)
{
  char text[64];
  static auto time_previous = std::chrono::steady_clock::now();
  auto time_now = std::chrono::steady_clock::now();
  double fps = 1e9 / (time_now - time_previous).count();
  time_previous = time_now;
  snprintf(text, sizeof(text), "FPS: %.1f, Inference: %.1f [ms]", fps, time_inference);
  CommonHelper::DrawText(mat, text, pos, font_scale, thickness, color_front, color_back, is_text_on_rect);
}

ClsMobileNetV2::ClsMobileNetV2(const rclcpp::NodeOptions & options)
: Node("ClsMobileNetV2", options)
{
  read_parameter();

  /*** For ROS stuffs ***/
  it_ = std::make_unique<image_transport::ImageTransport>(static_cast<rclcpp::Node::SharedPtr>(this)); 
  it_sub_ = it_->subscribe(prm_topic_name_image_sub_, 1, std::bind(&ClsMobileNetV2::image_callback, this, std::placeholders::_1));
  it_pub_ = it_->advertise(prm_topic_name_image_pub_, 1);

  publisher_result_ = this->create_publisher<inference_helper_sample_ros_interface::msg::Classification>(prm_topic_name_result_pub_, 10);

  /*** For image pocessing ***/
  engine_.reset(new ClassificationEngine());
  if (engine_->Initialize(prm_work_dir_, prm_thread_num_) != ClassificationEngine::kRetOk) {
      RCLCPP_ERROR(this->get_logger(), "Engine initialize error");
  }
}

ClsMobileNetV2::~ClsMobileNetV2()
{
  if (engine_->Finalize() != ClassificationEngine::kRetOk) {
      RCLCPP_ERROR(this->get_logger(), "Engine finalization error");
  }
  engine_.reset();
}

void ClsMobileNetV2::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  /*** For ROS stuffs ***/
  RCLCPP_INFO(this->get_logger(), "topic_image_callback: '%s'", msg->header.frame_id.c_str());

  // cv::Mat image_org = cv::Mat::zeros(256, 256, CV_8UC3);
  cv::Mat image_org = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
  // cv::imshow("image_org", image_org);

  /*** For image pocessing ***/
  cv::Mat image_result = image_org.clone();
  ClassificationEngine::Result cls_result;
  if (engine_->Process(image_org, cls_result) != ClassificationEngine::kRetOk) {
      RCLCPP_ERROR(this->get_logger(), "Engine calling error");
  }

  /* Draw the result */
  char text[64];
  snprintf(text, sizeof(text), "Result: %s (score = %.3f)",  cls_result.class_name.c_str(), cls_result.score);
  CommonHelper::DrawText(image_result, text, cv::Point(0, 20), 0.5, 2, CommonHelper::CreateCvColor(0, 0, 0), CommonHelper::CreateCvColor(180, 180, 180), true);

  DrawFps(image_result, cls_result.time_inference, cv::Point(0, 0), 0.5, 2, CommonHelper::CreateCvColor(0, 0, 0), CommonHelper::CreateCvColor(180, 180, 180), true);

  /*** For ROS stuffs ***/
  sensor_msgs::msg::Image::SharedPtr msg_img = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image_result).toImageMsg();
  it_pub_.publish(msg_img);

  auto msg_result = inference_helper_sample_ros_interface::msg::Classification();
  msg_result.header.stamp = this->get_clock()->now();
  msg_result.header.frame_id = "classification";
  msg_result.image_header = msg->header;
  msg_result.class_str = cls_result.class_name;
  msg_result.class_id = cls_result.class_id;
  msg_result.score = cls_result.score;
  publisher_result_->publish(msg_result);
}

void ClsMobileNetV2::read_parameter()
{
  this->declare_parameter<std::string>("work_dir", "/root/dev_ws/src/InferenceHelper_Sample_ROS/resource/");
  this->get_parameter("work_dir", prm_work_dir_);
  RCLCPP_INFO(this->get_logger(), "work_dir: %s", prm_work_dir_.c_str());

  this->declare_parameter<int32_t>("thread_num", 4);
  this->get_parameter("thread_num", prm_thread_num_);
  RCLCPP_INFO(this->get_logger(), "thread_num: %d", prm_thread_num_);

  this->declare_parameter<std::string>("topic_image_sub", "/image_raw");
  this->get_parameter("topic_image_sub", prm_topic_name_image_sub_);
  RCLCPP_INFO(this->get_logger(), "topic_image_sub: %s", prm_topic_name_image_sub_.c_str());

  this->declare_parameter<std::string>("topic_image_pub", "/transported_image_raw");
  this->get_parameter("topic_image_pub", prm_topic_name_image_pub_);
  RCLCPP_INFO(this->get_logger(), "topic_image_pub: %s", prm_topic_name_image_pub_.c_str());

  this->declare_parameter<std::string>("topic_result_pub", "/inference_result");
  this->get_parameter("topic_result_pub", prm_topic_name_result_pub_);
  RCLCPP_INFO(this->get_logger(), "topic_result_pub: %s", prm_topic_name_result_pub_.c_str());
}

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(inference_helper_sample_ros::ClsMobileNetV2)