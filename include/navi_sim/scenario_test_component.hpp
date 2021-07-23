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

#ifndef NAVI_SIM__SCENARIO_TEST_COMPONENT_HPP_
#define NAVI_SIM__SCENARIO_TEST_COMPONENT_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define NAVI_SIM_SCENARIO_TEST_COMPONENT_EXPORT __attribute__((dllexport))
#define NAVI_SIM_SCENARIO_TEST_COMPONENT_IMPORT __attribute__((dllimport))
#else
#define NAVI_SIM_SCENARIO_TEST_COMPONENT_EXPORT __declspec(dllexport)
#define NAVI_SIM_SCENARIO_TEST_COMPONENT_IMPORT __declspec(dllimport)
#endif
#ifdef NAVI_SIM_SCENARIO_TEST_COMPONENT_BUILDING_DLL
#define NAVI_SIM_SCENARIO_TEST_COMPONENT_PUBLIC NAVI_SIM_SCENARIO_TEST_COMPONENT_EXPORT
#else
#define NAVI_SIM_SCENARIO_TEST_COMPONENT_PUBLIC NAVI_SIM_SCENARIO_TEST_COMPONENT_IMPORT
#endif
#define NAVI_SIM_SCENARIO_TEST_COMPONENT_PUBLIC_TYPE NAVI_SIM_SCENARIO_TEST_COMPONENT_PUBLIC
#define NAVI_SIM_SCENARIO_TEST_COMPONENT_LOCAL
#else
#define NAVI_SIM_SCENARIO_TEST_COMPONENT_EXPORT __attribute__((visibility("default")))
#define NAVI_SIM_SCENARIO_TEST_COMPONENT_IMPORT
#if __GNUC__ >= 4
#define NAVI_SIM_SCENARIO_TEST_COMPONENT_PUBLIC __attribute__((visibility("default")))
#define NAVI_SIM_SCENARIO_TEST_COMPONENT_LOCAL __attribute__((visibility("hidden")))
#else
#define NAVI_SIM_SCENARIO_TEST_COMPONENT_PUBLIC
#define NAVI_SIM_SCENARIO_TEST_COMPONENT_LOCAL
#endif
#define NAVI_SIM_SCENARIO_TEST_COMPONENT_PUBLIC_TYPE
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
#include <navi_sim/interpreter/interpreter.hpp>

#include <boost/optional.hpp>

#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace navi_sim
{
class ScenarioTestComponent : public rclcpp::Node
{
public:
  NAVI_SIM_SCENARIO_TEST_COMPONENT_PUBLIC
  explicit ScenarioTestComponent(const rclcpp::NodeOptions & options);
  NAVI_SIM_SCENARIO_TEST_COMPONENT_PUBLIC
  explicit ScenarioTestComponent(std::string name, const rclcpp::NodeOptions & options);
  template<typename T, typename ... Ts>
  void addPrimitive(Ts && ... xs)
  {
    raycaster_ptr_->addPrimitive<T>(std::forward<Ts>(xs)...);
  }

private:
  bool checkCollision();
  bool checkCollision(
    const std::vector<geometry_msgs::msg::Point> & poly0,
    const std::vector<geometry_msgs::msg::Point> & poly1);
  void initialize();
  std::unique_ptr<navi_sim::Raycaster> raycaster_ptr_;
  double bbox_center_x_;
  double bbox_center_y_;
  double bbox_center_z_;
  double bbox_length_;
  double bbox_width_;
  double update_rate_;
  std::string map_frame_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  const std::vector<geometry_msgs::msg::Point> getBboxPolygon();
  const geometry_msgs::msg::Point getCornerPoint(bool x_dir, bool y_dir) const;
  std::vector<geometry_msgs::msg::Point> transformPoints(
    const geometry_msgs::msg::TransformStamped & pose,
    const std::vector<geometry_msgs::msg::Point> & points);
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr collision_marker_pub_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  void update();
  const visualization_msgs::msg::MarkerArray getCollisionMarker(bool collision);
  std::unique_ptr<navi_sim::Interpreter> interpreter_;
  std::string scenario_filename_;
  const boost::optional<geometry_msgs::msg::Pose> getEgoPose();
};
}  // namespace navi_sim

#endif  // NAVI_SIM__SCENARIO_TEST_COMPONENT_HPP_
