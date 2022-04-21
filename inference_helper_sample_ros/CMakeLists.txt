cmake_minimum_required(VERSION 3.5)
project(inference_helper_sample_ros)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(inference_helper_sample_ros_interface REQUIRED)

#####
add_library(cls_mobilenet_v2 SHARED src/cls_mobilenet_v2.cpp)
target_include_directories(cls_mobilenet_v2 PRIVATE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_definitions(cls_mobilenet_v2 PRIVATE "INFERENCE_HELPER_SAMPLE_FOR_ROS_BUILDING_DLL")
ament_target_dependencies(cls_mobilenet_v2
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
  "inference_helper_sample_ros_interface"
)
rclcpp_components_register_node(cls_mobilenet_v2 PLUGIN "inference_helper_sample_ros::ClsMobileNetV2" EXECUTABLE cls_mobilenet_v2_exe)
#####


install(TARGETS
  cls_mobilenet_v2
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
