// Copyright (c) 2021 OUXT Polaris
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

#ifndef NAVI_SIM__SEMANTIC_MAP_SIM_COMPONENT_HPP_
#define NAVI_SIM__SEMANTIC_MAP_SIM_COMPONENT_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define NAVI_SIM_SEMANTIC_MAP_SIM_COMPONENT_EXPORT __attribute__((dllexport))
#define NAVI_SIM_SEMANTIC_MAP_SIM_COMPONENT_IMPORT __attribute__((dllimport))
#else
#define NAVI_SIM_SEMANTIC_MAP_SIM_COMPONENT_EXPORT __declspec(dllexport)
#define NAVI_SIM_SEMANTIC_MAP_SIM_COMPONENT_IMPORT __declspec(dllimport)
#endif
#ifdef NAVI_SIM_SEMANTIC_MAP_SIM_COMPONENT_BUILDING_DLL
#define NAVI_SIM_SEMANTIC_MAP_SIM_COMPONENT_PUBLIC NAVI_SIM_SEMANTIC_MAP_SIM_COMPONENT_EXPORT
#else
#define NAVI_SIM_SEMANTIC_MAP_SIM_COMPONENT_PUBLIC NAVI_SIM_SEMANTIC_MAP_SIM_COMPONENT_IMPORT
#endif
#define NAVI_SIM_SEMANTIC_MAP_SIM_COMPONENT_PUBLIC_TYPE NAVI_SIM_SEMANTIC_MAP_SIM_COMPONENT_PUBLIC
#define NAVI_SIM_SEMANTIC_MAP_SIM_COMPONENT_LOCAL
#else
#define NAVI_SIM_SEMANTIC_MAP_SIM_COMPONENT_EXPORT __attribute__((visibility("default")))
#define NAVI_SIM_SEMANTIC_MAP_SIM_COMPONENT_IMPORT
#if __GNUC__ >= 4
#define NAVI_SIM_SEMANTIC_MAP_SIM_COMPONENT_PUBLIC __attribute__((visibility("default")))
#define NAVI_SIM_SEMANTIC_MAP_SIM_COMPONENT_LOCAL __attribute__((visibility("hidden")))
#else
#define NAVI_SIM_SEMANTIC_MAP_SIM_COMPONENT_PUBLIC
#define NAVI_SIM_SEMANTIC_MAP_SIM_COMPONENT_LOCAL
#endif
#define NAVI_SIM_SEMANTIC_MAP_SIM_COMPONENT_PUBLIC_TYPE
#endif
#if __cplusplus
}  // extern "C"
#endif

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <navi_sim/raycaster.hpp>
#include <rclcpp/rclcpp.hpp>
#include <robotx_behavior_msgs/msg/task_objects_array.hpp>

namespace navi_sim
{
class SemanticMapSimComponent : public rclcpp::Node
{
public:
  NAVI_SIM_SEMANTIC_MAP_SIM_COMPONENT_PUBLIC
  explicit SemanticMapSimComponent(const rclcpp::NodeOptions & options);
  NAVI_SIM_SEMANTIC_MAP_SIM_COMPONENT_PUBLIC
  explicit SemanticMapSimComponent(std::string name, const rclcpp::NodeOptions & options);
  template <typename T, typename... Ts>
  void addPrimitive(Ts &&... xs)
  {
    raycaster_ptr_->addPrimitive<T>(std::forward<Ts>(xs)...);
  }

private:
  void setParameters();
  void updateMap();
  rclcpp::Publisher<robotx_behavior_msgs::msg::TaskObjectsArray>::SharedPtr task_objects_pub_;
  std::unique_ptr<navi_sim::Raycaster> raycaster_ptr_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  rclcpp::TimerBase::SharedPtr update_map_timer_;
  std::string map_frame_, robot_frame_;
  double detection_distance_;
  std::string objects_path_;
  std::string task_objects_topic_;
  std::unordered_map<std::string, std::pair<size_t, rclcpp::Time> > tracked_objects_;
};
}  // namespace navi_sim

#endif  // NAVI_SIM__SEMANTIC_MAP_SIM_COMPONENT_HPP_
