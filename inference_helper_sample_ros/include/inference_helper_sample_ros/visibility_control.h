#ifndef INFERENCE_HELPER_SAMPLE_FOR_ROS__VISIBILITY_CONTROL_H_
#define INFERENCE_HELPER_SAMPLE_FOR_ROS__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define INFERENCE_HELPER_SAMPLE_FOR_ROS_EXPORT __attribute__ ((dllexport))
    #define INFERENCE_HELPER_SAMPLE_FOR_ROS_IMPORT __attribute__ ((dllimport))
  #else
    #define INFERENCE_HELPER_SAMPLE_FOR_ROS_EXPORT __declspec(dllexport)
    #define INFERENCE_HELPER_SAMPLE_FOR_ROS_IMPORT __declspec(dllimport)
  #endif
  #ifdef INFERENCE_HELPER_SAMPLE_FOR_ROS_BUILDING_DLL
    #define INFERENCE_HELPER_SAMPLE_FOR_ROS_PUBLIC INFERENCE_HELPER_SAMPLE_FOR_ROS_EXPORT
  #else
    #define INFERENCE_HELPER_SAMPLE_FOR_ROS_PUBLIC INFERENCE_HELPER_SAMPLE_FOR_ROS_IMPORT
  #endif
  #define INFERENCE_HELPER_SAMPLE_FOR_ROS_PUBLIC_TYPE INFERENCE_HELPER_SAMPLE_FOR_ROS_PUBLIC
  #define INFERENCE_HELPER_SAMPLE_FOR_ROS_LOCAL
#else
  #define INFERENCE_HELPER_SAMPLE_FOR_ROS_EXPORT __attribute__ ((visibility("default")))
  #define INFERENCE_HELPER_SAMPLE_FOR_ROS_IMPORT
  #if __GNUC__ >= 4
    #define INFERENCE_HELPER_SAMPLE_FOR_ROS_PUBLIC __attribute__ ((visibility("default")))
    #define INFERENCE_HELPER_SAMPLE_FOR_ROS_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define INFERENCE_HELPER_SAMPLE_FOR_ROS_PUBLIC
    #define INFERENCE_HELPER_SAMPLE_FOR_ROS_LOCAL
  #endif
  #define INFERENCE_HELPER_SAMPLE_FOR_ROS_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // INFERENCE_HELPER_SAMPLE_FOR_ROS__VISIBILITY_CONTROL_H_