#include "rclcpp/rclcpp.hpp"
#include "image_transport/image_transport.hpp"
#include "inference_helper_sample_ros/visibility_control.h"
#include "inference_helper_sample_ros_interface/msg/classification.hpp"

class ClassificationEngine;

namespace inference_helper_sample_ros
{

class ClsMobileNetV2 : public rclcpp::Node
{
public:
  INFERENCE_HELPER_SAMPLE_FOR_ROS_PUBLIC explicit ClsMobileNetV2(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());

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

  rclcpp::Publisher<inference_helper_sample_ros_interface::msg::Classification>::SharedPtr publisher_result_;

  std::unique_ptr<ClassificationEngine> classification_engine_;

};

}
