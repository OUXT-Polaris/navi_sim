#ifndef NAVI_SIM_NAVI_SIM_COMPONENT_H_INCLUDED
#define NAVI_SIM_NAVI_SIM_COMPONENT_H_INCLUDED

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define NAVI_SIM_NAVI_SIM_COMPONENT_EXPORT __attribute__ ((dllexport))
    #define NAVI_SIM_NAVI_SIM_COMPONENT_IMPORT __attribute__ ((dllimport))
  #else
    #define NAVI_SIM_NAVI_SIM_COMPONENT_EXPORT __declspec(dllexport)
    #define NAVI_SIM_NAVI_SIM_COMPONENT_IMPORT __declspec(dllimport)
  #endif
  #ifdef NAVI_SIM_NAVI_SIM_COMPONENT_BUILDING_DLL
    #define NAVI_SIM_NAVI_SIM_COMPONENT_PUBLIC NAVI_SIM_NAVI_SIM_COMPONENT_EXPORT
  #else
    #define NAVI_SIM_NAVI_SIM_COMPONENT_PUBLIC NAVI_SIM_NAVI_SIM_COMPONENT_IMPORT
  #endif
  #define NAVI_SIM_NAVI_SIM_COMPONENT_PUBLIC_TYPE NAVI_SIM_NAVI_SIM_COMPONENT_PUBLIC
  #define NAVI_SIM_NAVI_SIM_COMPONENT_LOCAL
#else
  #define NAVI_SIM_NAVI_SIM_COMPONENT_EXPORT __attribute__ ((visibility("default")))
  #define NAVI_SIM_NAVI_SIM_COMPONENT_IMPORT
  #if __GNUC__ >= 4
    #define NAVI_SIM_NAVI_SIM_COMPONENT_PUBLIC __attribute__ ((visibility("default")))
    #define NAVI_SIM_NAVI_SIM_COMPONENT_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define NAVI_SIM_NAVI_SIM_COMPONENT_PUBLIC
    #define NAVI_SIM_NAVI_SIM_COMPONENT_LOCAL
  #endif
  #define NAVI_SIM_NAVI_SIM_COMPONENT_PUBLIC_TYPE
#endif
#if __cplusplus
} // extern "C"
#endif

// Headers in ROS2
#include <rclcpp/rclcpp.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

// Headers in STL
#include <chrono>
#include <mutex>

#include <quaternion_operation/quaternion_operation.h>

namespace navi_sim
{
  class NaviSimComponent: public rclcpp::Node
  {
    public:
      NAVI_SIM_NAVI_SIM_COMPONENT_PUBLIC
      explicit NaviSimComponent(const rclcpp::NodeOptions & options);
    private:
      rclcpp::Clock ros_clock_;
      tf2_ros::TransformBroadcaster broadcaster_;
      rclcpp::TimerBase::SharedPtr update_position_timer_;
      geometry_msgs::msg::Twist current_twist_;
      geometry_msgs::msg::Pose current_pose_;
      void updatePose();
      void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr data);
      rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
      void targetTwistCallback(const geometry_msgs::msg::Twist::SharedPtr data);
      rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr target_twist_sub_;
      std::mutex mtx_;
  };
}

#endif  //NAVI_SIM_NAVI_SIM_COMPONENT_H_INCLUDED