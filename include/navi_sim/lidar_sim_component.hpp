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

#ifndef NAVI_SIM__LIDAR_SIM_COMPONENT_HPP_
#define NAVI_SIM__LIDAR_SIM_COMPONENT_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define NAVI_SIM_LIDAR_SIM_COMPONENT_EXPORT __attribute__((dllexport))
#define NAVI_SIM_LIDAR_SIM_COMPONENT_IMPORT __attribute__((dllimport))
#else
#define NAVI_SIM_LIDAR_SIM_COMPONENT_EXPORT __declspec(dllexport)
#define NAVI_SIM_LIDAR_SIM_COMPONENT_IMPORT __declspec(dllimport)
#endif
#ifdef NAVI_SIM_LIDAR_SIM_COMPONENT_BUILDING_DLL
#define NAVI_SIM_LIDAR_SIM_COMPONENT_PUBLIC NAVI_SIM_LIDAR_SIM_COMPONENT_EXPORT
#else
#define NAVI_SIM_LIDAR_SIM_COMPONENT_PUBLIC NAVI_SIM_LIDAR_SIM_COMPONENT_IMPORT
#endif
#define NAVI_SIM_LIDAR_SIM_COMPONENT_PUBLIC_TYPE NAVI_SIM_LIDAR_SIM_COMPONENT_PUBLIC
#define NAVI_SIM_LIDAR_SIM_COMPONENT_LOCAL
#else
#define NAVI_SIM_LIDAR_SIM_COMPONENT_EXPORT __attribute__((visibility("default")))
#define NAVI_SIM_LIDAR_SIM_COMPONENT_IMPORT
#if __GNUC__ >= 4
#define NAVI_SIM_LIDAR_SIM_COMPONENT_PUBLIC __attribute__((visibility("default")))
#define NAVI_SIM_LIDAR_SIM_COMPONENT_LOCAL __attribute__((visibility("hidden")))
#else
#define NAVI_SIM_LIDAR_SIM_COMPONENT_PUBLIC
#define NAVI_SIM_LIDAR_SIM_COMPONENT_LOCAL
#endif
#define NAVI_SIM_LIDAR_SIM_COMPONENT_PUBLIC_TYPE
#endif
#if __cplusplus
}  // extern "C"
#endif

#include <navi_sim/raycaster.hpp>

#include <rclcpp/rclcpp.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <sensor_msgs/msg/point_cloud2.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include <memory>
#include <string>
#include <utility>

namespace navi_sim
{
class LidarSimComponent : public rclcpp::Node
{
public:
  NAVI_SIM_LIDAR_SIM_COMPONENT_PUBLIC
  explicit LidarSimComponent(const rclcpp::NodeOptions & options);
  template<typename T, typename ... Ts>
  void addPrimitive(Ts && ... xs)
  {
    raycaster_ptr_->addPrimitive<T>(std::forward<Ts>(xs)...);
  }

private:
  void updateScan();
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  // rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  std::unique_ptr<navi_sim::Raycaster> raycaster_ptr_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  rclcpp::TimerBase::SharedPtr update_scan_timer_;
  std::string lidar_frame_, map_frame_;
};
}  // namespace navi_sim

#endif  // NAVI_SIM__LIDAR_SIM_COMPONENT_HPP_
