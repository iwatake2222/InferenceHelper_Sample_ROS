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

#include "inference_helper_sample_ros/depth_lapdepth.hpp"

#include "common_helper.h"
#include "common_helper_cv.h"
#include "engine/engine_depth_lapdepth.hpp"

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

DepthLapdepth::DepthLapdepth(const rclcpp::NodeOptions & options)
: Node("DepthLapdepth", options)
{
  read_parameter();

  /*** For ROS stuffs ***/
  it_ = std::make_unique<image_transport::ImageTransport>(static_cast<rclcpp::Node::SharedPtr>(this)); 
  it_sub_ = it_->subscribe(prm_topic_name_image_sub_, 1, std::bind(&DepthLapdepth::image_callback, this, std::placeholders::_1));
  it_pub_ = it_->advertise(prm_topic_name_image_pub_, 1);

  publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(prm_topic_name_result_pub_, 10);

  /*** For image pocessing ***/
  engine_.reset(new DepthEngine());
  if (engine_->Initialize(prm_work_dir_, prm_thread_num_) != DepthEngine::kRetOk) {
      RCLCPP_ERROR(this->get_logger(), "Engine initialize error");
  }
}

DepthLapdepth::~DepthLapdepth()
{
  if (engine_->Finalize() != DepthEngine::kRetOk) {
      RCLCPP_ERROR(this->get_logger(), "Engine finalization error");
  }
  engine_.reset();
}

void DepthLapdepth::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  /*** For ROS stuffs ***/
  RCLCPP_INFO(this->get_logger(), "topic_image_callback: '%s'", msg->header.frame_id.c_str());

  cv::Mat image_org = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;

  /*** For image pocessing ***/
  DepthEngine::Result depth_result;
  if (engine_->Process(image_org, depth_result) != DepthEngine::kRetOk) {
      RCLCPP_ERROR(this->get_logger(), "Engine calling error");
  }

  /* Draw the result and generate point cloud*/
  cv::Mat image_result;
  std::vector<cv::Point3f> object_point_list;
  if (!depth_result.mat_out.empty()) {
    cv::Mat depth_org = depth_result.mat_out;

    cv::Mat depth_normlized;
    double depth_min, depth_max;
    cv::minMaxLoc(depth_org, &depth_min, &depth_max);
    depth_org.convertTo(depth_normlized, CV_8UC1, 255. / (depth_max - depth_min), (-255. * depth_min) / (depth_max - depth_min));
#if 1
    cv::applyColorMap(depth_normlized, image_result, cv::COLORMAP_MAGMA);
#else
    depth_org.convertTo(image_result, CV_8UC1);
    cv::cvtColor(image_result, image_result, cv::COLOR_GRAY2BGR);
#endif
    // image_result = image_result(cv::Rect(0, static_cast<int32_t>(image_result.rows * 0.18), image_result.cols, static_cast<int32_t>(image_result.rows * (1.0 - 0.18))));

    /* Generate depth list */
    // depth_normlized = 255 - depth_normlized;
    cv::resize(depth_normlized, depth_normlized, image_org.size());
    std::vector<float> depth_list;
    for (int32_t y = 0; y < depth_normlized.rows; y++) {
        for (int32_t x = 0; x < depth_normlized.cols; x++) {
            float Z = depth_normlized.at<uint8_t>(cv::Point(x, y));
            depth_list.push_back(Z);
        }
    }

    /* Convert px,py,depth(Zc) -> Xc,Yc,Zc(in camera_2d_to_3d)(=Xw,Yw,Zw) */
    camera_2d_to_3d_.SetIntrinsic(depth_normlized.cols, depth_normlized.rows, FocalLength(depth_normlized.cols, 60));
    camera_2d_to_3d_.SetExtrinsic(
      { 0.0f, 0.0f, 0.0f },    /* rvec [deg] */
      { 0.0f, 0.0f, 0.0f }, true);
    std::vector<cv::Point2f> image_point_list;  // empty
    camera_2d_to_3d_.ConvertImage2Camera(image_point_list, depth_list, object_point_list);
  }

  DrawFps(image_result, depth_result.time_inference, cv::Point(0, 0), 0.5, 2, CommonHelper::CreateCvColor(0, 0, 0), CommonHelper::CreateCvColor(180, 180, 180), true);

  /*** For ROS stuffs ***/
  sensor_msgs::msg::Image::SharedPtr msg_img = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image_result).toImageMsg();
  it_pub_.publish(msg_img);
  
  auto msg_pc = sensor_msgs::msg::PointCloud2();
  // make_pc2_msg(object_point_list, image_org.cols, image_org.rows, msg_pc);
  make_pc2_color_msg(object_point_list, image_org.data, image_org.cols, image_org.rows, msg_pc);
  publisher_->publish(msg_pc);
}

void DepthLapdepth::read_parameter()
{
  this->declare_parameter<std::string>("work_dir", "./src/InferenceHelper_Sample_ROS/resource/");
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


void DepthLapdepth::make_pc2_msg(const std::vector<cv::Point3f>& object_list, int32_t width, int32_t height, sensor_msgs::msg::PointCloud2 & msg)
{
  static const size_t POINT_CLOUD_POINT_SIZE = 3;
  static const size_t POINT_CLOUD_ELEM_SIZE = sizeof(float);

  msg.header.frame_id = "map";
  msg.header.stamp = this->get_clock()->now();
  msg.height = height;
  msg.width = width;
  msg.fields.resize(3);
  msg.fields[0].set__name("x");
  msg.fields[0].set__datatype(sensor_msgs::msg::PointField::FLOAT32);
  msg.fields[0].set__offset(0);
  msg.fields[0].set__count(1);
  msg.fields[1].set__name("y");
  msg.fields[1].set__datatype(sensor_msgs::msg::PointField::FLOAT32);
  msg.fields[1].set__offset(4);
  msg.fields[1].set__count(1);
  msg.fields[2].set__name("z");
  msg.fields[2].set__datatype(sensor_msgs::msg::PointField::FLOAT32);
  msg.fields[2].set__offset(8);
  msg.fields[2].set__count(1);

  msg.is_bigendian = false;
  msg.point_step = POINT_CLOUD_POINT_SIZE * POINT_CLOUD_ELEM_SIZE;
  msg.row_step = msg.point_step * msg.width;
  msg.data.resize(msg.row_step * msg.height);

  for (size_t y = 0; y < msg.height; y++) {
    for (size_t x = 0; x < msg.width; x++) {
      size_t offset_src = y * msg.width + x;
      const auto& src_xyz = object_list[offset_src];
      size_t offset_dst = (y * (msg.width) + x) * POINT_CLOUD_POINT_SIZE * POINT_CLOUD_ELEM_SIZE;
      float * dst = (float*)(&msg.data[offset_dst]);
      dst[0] = src_xyz.z;
      dst[1] = -src_xyz.x;
      dst[2] = -src_xyz.y;
    }
  }

  msg.is_dense = false;
}

void DepthLapdepth::make_pc2_color_msg(const std::vector<cv::Point3f>& object_list, const uint8_t color_list[], int32_t width, int32_t height, sensor_msgs::msg::PointCloud2 & msg)
{
  static const size_t POINT_CLOUD_POINT_SIZE = 4;
  static const size_t POINT_CLOUD_ELEM_SIZE = 4;

  msg.header.frame_id = "map";
  msg.header.stamp = this->get_clock()->now();
  msg.height = height;
  msg.width = width;
  msg.fields.resize(4);
  msg.fields[0].set__name("x");
  msg.fields[0].set__datatype(sensor_msgs::msg::PointField::FLOAT32);
  msg.fields[0].set__offset(0);
  msg.fields[0].set__count(1);
  msg.fields[1].set__name("y");
  msg.fields[1].set__datatype(sensor_msgs::msg::PointField::FLOAT32);
  msg.fields[1].set__offset(4);
  msg.fields[1].set__count(1);
  msg.fields[2].set__name("z");
  msg.fields[2].set__datatype(sensor_msgs::msg::PointField::FLOAT32);
  msg.fields[2].set__offset(8);
  msg.fields[2].set__count(1);
  msg.fields[3].set__name("rgba");
  msg.fields[3].set__datatype(sensor_msgs::msg::PointField::UINT32);
  msg.fields[3].set__offset(12);
  msg.fields[3].set__count(1);

  msg.is_bigendian = false;
  msg.point_step = POINT_CLOUD_POINT_SIZE * POINT_CLOUD_ELEM_SIZE;
  msg.row_step = msg.point_step * msg.width;
  msg.data.resize(msg.row_step * msg.height);

  for (size_t y = 0; y < msg.height; y++) {
    for (size_t x = 0; x < msg.width; x++) {
      size_t offset_src = y * msg.width + x;
      const auto& src_xyz = object_list[offset_src];
      const uint8_t * src_rgb = &color_list[offset_src * 3];
      size_t offset_dst = (y * (msg.width) + x) * POINT_CLOUD_POINT_SIZE * POINT_CLOUD_ELEM_SIZE;
      float * dst_float = (float*)(&msg.data[offset_dst]);
      uint32_t * dst_uint32 = (uint32_t*)(&msg.data[offset_dst]);
      dst_float[0] = src_xyz.z;
      dst_float[1] = -src_xyz.x;
      dst_float[2] = -src_xyz.y;
      uint8_t r = src_rgb[2];
      uint8_t g = src_rgb[1];
      uint8_t b = src_rgb[0];
      uint8_t a = 255;
      dst_uint32[3] =  (a << 24) | (r << 16) | (g << 8) | b;
    }
  }

  msg.is_dense = false;
}

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(inference_helper_sample_ros::DepthLapdepth)