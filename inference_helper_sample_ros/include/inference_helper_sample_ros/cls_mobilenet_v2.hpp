#include "rclcpp/rclcpp.hpp"

#include "inference_helper_sample_ros/visibility_control.h"
#include "inference_helper_sample_ros_interface/msg/num.hpp"

namespace inference_helper_sample_ros
{

class ClsMobileNetV2 : public rclcpp::Node
{
public:
  INFERENCE_HELPER_SAMPLE_FOR_ROS_PUBLIC explicit ClsMobileNetV2(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

private:
  void read_parameter();
  void timer_callback();

private:
  std::string parameter_string_;
  rclcpp::Publisher<inference_helper_sample_ros_interface::msg::Num>::SharedPtr publisher_;
  rclcpp::TimerBase::SharedPtr timer_;
  size_t count_;

};

}
