#include <cstdlib>
#include <chrono>
#include <memory>

#include "opencv2/opencv.hpp"

#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "cv_bridge/cv_bridge.h"

#include "inference_helper_sample_ros/cls_mobilenet_v2.hpp"

namespace inference_helper_sample_ros
{

ClsMobileNetV2::ClsMobileNetV2(const rclcpp::NodeOptions & options)
: Node("ClsMobileNetV2", options)
{
  read_parameter();

  it_ = std::make_unique<image_transport::ImageTransport>(static_cast<rclcpp::Node::SharedPtr>(this)); 
  it_sub_ = it_->subscribe(prm_topic_name_image_sub_, 1, std::bind(&ClsMobileNetV2::image_callback, this, std::placeholders::_1));
  it_pub_ = it_->advertise(prm_topic_name_image_pub_, 1);
}

void ClsMobileNetV2::image_callback(const sensor_msgs::msg::Image::ConstSharedPtr msg)
{
  RCLCPP_INFO(this->get_logger(), "topic_image_callback: '%s'", msg->header.frame_id.c_str());

  // cv::Mat image_org = cv::Mat::zeros(256, 256, CV_8UC3);
  cv::Mat image_org = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
  cv::imshow("image", image_org);
  cv::waitKey(1);

  cv::Mat image_pub;
  cv::resize(image_org, image_pub, cv::Size(), 0.2, 0.2);
  sensor_msgs::msg::Image::SharedPtr msg_pub = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image_pub).toImageMsg();
  it_pub_.publish(msg_pub);
}

void ClsMobileNetV2::read_parameter()
{
  this->declare_parameter<std::string>("topic_image_sub", "/image_raw");
  this->get_parameter("topic_image_sub", prm_topic_name_image_sub_);
  RCLCPP_INFO(this->get_logger(), "topic_image_sub: %s", prm_topic_name_image_sub_.c_str());

  this->declare_parameter<std::string>("topic_image_pub", "/transported_image_raw");
  this->get_parameter("topic_image_pub", prm_topic_name_image_pub_);
  RCLCPP_INFO(this->get_logger(), "topic_image_pub: %s", prm_topic_name_image_pub_.c_str());
}

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(inference_helper_sample_ros::ClsMobileNetV2)