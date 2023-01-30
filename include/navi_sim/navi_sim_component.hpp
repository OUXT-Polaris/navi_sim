// Copyright (c) 2020 OUXT Polaris
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef NAVI_SIM__NAVI_SIM_COMPONENT_HPP_
#define NAVI_SIM__NAVI_SIM_COMPONENT_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define NAVI_SIM_NAVI_SIM_COMPONENT_EXPORT __attribute__((dllexport))
#define NAVI_SIM_NAVI_SIM_COMPONENT_IMPORT __attribute__((dllimport))
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
#define NAVI_SIM_NAVI_SIM_COMPONENT_EXPORT __attribute__((visibility("default")))
#define NAVI_SIM_NAVI_SIM_COMPONENT_IMPORT
#if __GNUC__ >= 4
#define NAVI_SIM_NAVI_SIM_COMPONENT_PUBLIC __attribute__((visibility("default")))
#define NAVI_SIM_NAVI_SIM_COMPONENT_LOCAL __attribute__((visibility("hidden")))
#else
#define NAVI_SIM_NAVI_SIM_COMPONENT_PUBLIC
#define NAVI_SIM_NAVI_SIM_COMPONENT_LOCAL
#endif
#define NAVI_SIM_NAVI_SIM_COMPONENT_PUBLIC_TYPE
#endif
#if __cplusplus
}  // extern "C"
#endif

// Headers in ROS2
#include <quaternion_operation/quaternion_operation.h>

#ifdef USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/twist_stamped.hpp>
#include <geometry_msgs/msg/twist_with_covariance_stamped.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

// Headers in Boost
#include <boost/optional.hpp>

// Headers in STL
#include <chrono>
#include <mutex>
#include <random>
#include <vector>

namespace navi_sim
{
class NaviSimComponent : public rclcpp::Node
{
public:
  NAVI_SIM_NAVI_SIM_COMPONENT_PUBLIC
  explicit NaviSimComponent(const rclcpp::NodeOptions & options);

private:
  tf2_ros::TransformBroadcaster broadcaster_;
  rclcpp::TimerBase::SharedPtr update_position_timer_;
  geometry_msgs::msg::Twist current_twist_;
  geometry_msgs::msg::Pose current_pose_;
  void updatePose();
  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr data);
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  void targetTwistCallback(const geometry_msgs::msg::Twist::SharedPtr data);
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr target_twist_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
    current_pose_with_covariance_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr current_twist_pub_;
  rclcpp::Publisher<geometry_msgs::msg::TwistWithCovarianceStamped>::SharedPtr
    current_twist_with_covariance_pub_;
  rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  geometry_msgs::msg::PointStamped TransformToMapFrame(geometry_msgs::msg::PointStamped point);
  geometry_msgs::msg::PointStamped TransformToBaselinkFrame(
    geometry_msgs::msg::PointStamped point, bool from_message_timestamp = true);
  geometry_msgs::msg::PoseWithCovarianceStamped applyNoise(
    const geometry_msgs::msg::PoseStamped & pose);
  geometry_msgs::msg::TwistWithCovarianceStamped applyNoise(
    const geometry_msgs::msg::TwistStamped & twist);
  void updateJointState();
  bool with_covariance_;
  bool publish_twist_;
  bool publish_pose_;
  double position_covariance_;
  double rpy_covariance_;
  double linear_covariance_;
  double angular_covariance_;
  std::random_device seed_gen_;
  std::default_random_engine engine_;
};
}  // namespace navi_sim

#endif  // NAVI_SIM__NAVI_SIM_COMPONENT_HPP_
