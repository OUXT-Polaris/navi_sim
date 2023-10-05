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

#ifndef NAVI_SIM__CAMERA_SIM_COMPONENT_HPP_
#define NAVI_SIM__CAMERA_SIM_COMPONENT_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define NAVI_SIM_CAMEAR_SIM_COMPONENT_EXPORT __attribute__((dllexport))
#define NAVI_SIM_CAMEAR_SIM_COMPONENT_IMPORT __attribute__((dllimport))
#else
#define NAVI_SIM_CAMEAR_SIM_COMPONENT_EXPORT __declspec(dllexport)
#define NAVI_SIM_CAMEAR_SIM_COMPONENT_IMPORT __declspec(dllimport)
#endif
#ifdef NAVI_SIM_CAMEAR_SIM_COMPONENT_BUILDING_DLL
#define NAVI_SIM_CAMEAR_SIM_COMPONENT_PUBLIC NAVI_SIM_CAMEAR_SIM_COMPONENT_EXPORT
#else
#define NAVI_SIM_CAMEAR_SIM_COMPONENT_PUBLIC NAVI_SIM_CAMEAR_SIM_COMPONENT_IMPORT
#endif
#define NAVI_SIM_CAMEAR_SIM_COMPONENT_PUBLIC_TYPE NAVI_SIM_CAMEAR_SIM_COMPONENT_PUBLIC
#define NAVI_SIM_CAMEAR_SIM_COMPONENT_LOCAL
#else
#define NAVI_SIM_CAMEAR_SIM_COMPONENT_EXPORT __attribute__((visibility("default")))
#define NAVI_SIM_CAMEAR_SIM_COMPONENT_IMPORT
#if __GNUC__ >= 4
#define NAVI_SIM_CAMEAR_SIM_COMPONENT_PUBLIC __attribute__((visibility("default")))
#define NAVI_SIM_CAMEAR_SIM_COMPONENT_LOCAL __attribute__((visibility("hidden")))
#else
#define NAVI_SIM_CAMEAR_SIM_COMPONENT_PUBLIC
#define NAVI_SIM_CAMEAR_SIM_COMPONENT_LOCAL
#endif
#define NAVI_SIM_CAMEAR_SIM_COMPONENT_PUBLIC_TYPE
#endif
#if __cplusplus
}  // extern "C"
#endif

#include <image_geometry/pinhole_camera_model.h>

#ifdef USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#else
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#endif

#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

#include <memory>
#include <navi_sim/raycaster.hpp>
#include <perception_msgs/msg/detection2_d.hpp>
#include <perception_msgs/msg/detection2_d_array.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <string>
#include <utility>
#include <vector>
#include <visualization_msgs/msg/marker_array.hpp>

namespace navi_sim
{
class CameraSimComponent : public rclcpp::Node
{
public:
  NAVI_SIM_CAMEAR_SIM_COMPONENT_PUBLIC
  explicit CameraSimComponent(const rclcpp::NodeOptions & options);
  NAVI_SIM_CAMEAR_SIM_COMPONENT_PUBLIC
  explicit CameraSimComponent(std::string name, const rclcpp::NodeOptions & options);
  template <typename T, typename... Ts>
  void addPrimitive(Ts &&... xs)
  {
    raycaster_ptr_->addPrimitive<T>(std::forward<Ts>(xs)...);
  }

private:
  void update();
  void initialize();
  unique_identifier_msgs::msg::UUID generateUUID(const std::string & seed) const;
  std::unique_ptr<navi_sim::Raycaster> raycaster_ptr_;
  sensor_msgs::msg::CameraInfo camera_info_;
  image_geometry::PinholeCameraModel cam_model_;
  rclcpp::TimerBase::SharedPtr update_camera_timer_;
  rclcpp::Publisher<perception_msgs::msg::Detection2DArray>::SharedPtr detection_pub_;
  rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  tf2_ros::Buffer buffer_;
  tf2_ros::TransformListener listener_;
  std::string map_frame_;
  std::string camera_optical_frame_;
  std::string camera_frame_;
  std::string frustum_color_;
  std::string detection_color_;
  int vertical_pixels_;
  int horizontal_pixels_;
  const visualization_msgs::msg::MarkerArray generateMarker(
    const std::vector<perception_msgs::msg::Detection2D> & detections);
  const geometry_msgs::msg::Point internallyDivide(
    const geometry_msgs::msg::Point & p0, const geometry_msgs::msg::Point & p1,
    double x_ratio_in_image, double y_ratio_in_image);
};
}  // namespace navi_sim

#endif  // NAVI_SIM__CAMERA_SIM_COMPONENT_HPP_
