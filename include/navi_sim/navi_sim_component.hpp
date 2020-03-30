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
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

// Headers in STL
#include <chrono>
#include <mutex>

#include <quaternion_operation/quaternion_operation.h>

// Headers in Boost
#include <boost/optional.hpp>

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
  void clickedPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr data);
  void initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr data);
  rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr initial_pose_sub_;
  void targetTwistCallback(const geometry_msgs::msg::Twist::SharedPtr data);
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr clicked_point_sub_;
  std::vector<geometry_msgs::msg::Point> obstacles_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr target_twist_sub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr current_pose_pub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr current_twist_pub_;
  rclcpp::Publisher<sensor_msgs::msg::LaserScan>::SharedPtr laser_pub_;
  std::mutex mtx_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  geometry_msgs::msg::PointStamped TransformToMapFrame(geometry_msgs::msg::PointStamped point);
  geometry_msgs::msg::PointStamped TransformToBaselinkFrame(
    geometry_msgs::msg::PointStamped point,
    bool from_message_timestamp = true);
  double obstacle_radius_;
  double maximum_scan_range_;
  double minimum_scan_range_;
  int num_scans_;
  void updateScan();
  rclcpp::TimerBase::SharedPtr update_scan_timer_;
  boost::optional<double> getDistanceToObstacle(
    geometry_msgs::msg::PointStamped obstacle,
    double theta);
};
}

#endif  //NAVI_SIM_NAVI_SIM_COMPONENT_H_INCLUDED
