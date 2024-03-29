cmake_minimum_required(VERSION 3.5)
project(inference_helper_sample_ros)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

##### Compile settings #####
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Select build system for Inference Helper
set(BUILD_SYSTEM auto CACHE STRING "Build target? [auto, x64_windows, x64_linux, armv7, aarch64]")
if(${BUILD_SYSTEM} STREQUAL "auto")
    if(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "armv7l")
        set(BUILD_SYSTEM armv7)
    elseif(${CMAKE_SYSTEM_PROCESSOR} STREQUAL "aarch64")
        set(BUILD_SYSTEM aarch64)
    else()
        if(WIN32)
            set(BUILD_SYSTEM x64_windows)
        else()
            set(BUILD_SYSTEM x64_linux)
        endif()
    endif()
endif()
message("[main] CMAKE_SYSTEM_PROCESSOR = " ${CMAKE_SYSTEM_PROCESSOR} ", BUILD_SYSTEM = " ${BUILD_SYSTEM})

##### Dependencies #####
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(inference_helper_sample_ros_interface REQUIRED)
find_package(OpenCV REQUIRED)
find_package(OpenMP)
if(OPENMP_FOUND)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    # set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fopenmp")
    # set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fopenmp")
endif()

# For InferenceHelper and Common Helper module
set(INFERENCE_HELPER_DIR ${CMAKE_CURRENT_SOURCE_DIR}/src/inference_helper)
set(COMMON_HELPER_DIR ${CMAKE_CURRENT_LIST_DIR}/src/common_helper)
set(BACKPU_CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}")   # patch (to run with component)
set(CMAKE_CXX_FLAGS "-fPIC ${CMAKE_CXX_FLAGS}")
add_subdirectory(${INFERENCE_HELPER_DIR}/inference_helper inference_helper)
add_subdirectory(${CMAKE_CURRENT_LIST_DIR}/src/common_helper common_helper)
set(CMAKE_CXX_FLAGS "${BACKPU_CMAKE_CXX_FLAGS}")

##### Function to create module #####
function(create_module_func LIB_NAME COMPONENT_NAME SOURCES)
  cmake_parse_arguments(PARSE_ARGV 0 "MYFUNC"  "" "" "SOURCES")
  message("LIB_NAME = ${LIB_NAME}")
  message("COMPONENT_NAME = ${COMPONENT_NAME}")
  message("SOURCES = ${MYFUNC_SOURCES}")

  include(inference_helper_def.cmake)
  add_library(${LIB_NAME} SHARED ${MYFUNC_SOURCES})
  target_include_directories(${LIB_NAME} PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    ${INFERENCE_HELPER_DIR}/inference_helper
    ${COMMON_HELPER_DIR})
  target_link_libraries("${LIB_NAME}" CommonHelper InferenceHelper)
  target_compile_definitions(${LIB_NAME} PRIVATE "INFERENCE_HELPER_SAMPLE_FOR_ROS_BUILDING_DLL")
  ament_target_dependencies(${LIB_NAME}
    "rclcpp"
    "rclcpp_components"
    "std_msgs"
    "sensor_msgs"
    "cv_bridge"
    "image_transport"
    "OpenCV"
    "inference_helper_sample_ros_interface"
  )
  rclcpp_components_register_node("${LIB_NAME}" PLUGIN "inference_helper_sample_ros::${COMPONENT_NAME}" EXECUTABLE "${LIB_NAME}_exe")

  install(TARGETS
    ${LIB_NAME}
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin)
endfunction()


##### Modules #####
create_module_func("transport" "Transport" SOURCES "src/transport.cpp" "include/inference_helper_sample_ros/transport.hpp")
create_module_func("cls_mobilenet_v2" "ClsMobileNetV2" SOURCES "src/cls_mobilenet_v2.cpp" "include/inference_helper_sample_ros/cls_mobilenet_v2.hpp"
    "src/engine/engine_cls_mobilenet_v2.cpp" "src/engine/engine_cls_mobilenet_v2.hpp")
create_module_func("det_yolox" "DetYolox" SOURCES "src/det_yolox.cpp" "include/inference_helper_sample_ros/det_yolox.hpp"
    "src/engine/engine_det_yolox.cpp" "src/engine/engine_det_yolox.hpp")
create_module_func("seg_paddleseg_cityscapessota" "SegPaddlesegCityscapessota" SOURCES "src/seg_paddleseg_cityscapessota.cpp" "include/inference_helper_sample_ros/seg_paddleseg_cityscapessota.hpp"
    "src/engine/engine_seg_paddleseg_cityscapessota.cpp" "src/engine/engine_seg_paddleseg_cityscapessota.hpp")
create_module_func("depth_lapdepth" "DepthLapdepth" SOURCES "src/depth_lapdepth.cpp" "include/inference_helper_sample_ros/depth_lapdepth.hpp"
    "src/engine/engine_depth_lapdepth.cpp" "src/engine/engine_depth_lapdepth.hpp")

# Memo: for some reasons, I need these lines to create component even though I did rclcpp_components_register_node above...
rclcpp_components_register_nodes("transport" "inference_helper_sample_ros::Transport")
rclcpp_components_register_nodes("cls_mobilenet_v2" "inference_helper_sample_ros::ClsMobileNetV2")
rclcpp_components_register_nodes("det_yolox" "inference_helper_sample_ros::DetYolox")
rclcpp_components_register_nodes("seg_paddleseg_cityscapessota" "inference_helper_sample_ros::SegPaddlesegCityscapessota")
rclcpp_components_register_nodes("depth_lapdepth" "inference_helper_sample_ros::DepthLapdepth")

##### Install #####
install(
  DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
