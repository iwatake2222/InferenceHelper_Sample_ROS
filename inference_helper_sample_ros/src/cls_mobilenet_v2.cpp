#include <cstdlib>
#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"

#include "inference_helper_sample_ros/cls_mobilenet_v2.hpp"

namespace inference_helper_sample_ros
{

ClsMobileNetV2::ClsMobileNetV2(const rclcpp::NodeOptions & options)
: Node("cls_mobilenet_v2", options)
{
  read_parameter();
  publisher_ = this->create_publisher<inference_helper_sample_ros_interface::msg::Num>("my_topic", 10);
  timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&ClsMobileNetV2::timer_callback, this));
}

void ClsMobileNetV2::timer_callback()
{
  auto msg = inference_helper_sample_ros_interface::msg::Num();
  msg.num = count_++;
  RCLCPP_INFO(this->get_logger(), "[timer_callback] Publishing: '%d'", msg.num);
  publisher_->publish(msg);
}

void ClsMobileNetV2::read_parameter()
{
  this->declare_parameter<std::string>("dummy", "Default Parameter");
  this->get_parameter("dummy", parameter_string_);
  RCLCPP_INFO(this->get_logger(), "dummy: %s", parameter_string_.c_str());
}

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(inference_helper_sample_ros::ClsMobileNetV2)