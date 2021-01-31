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

#include <navi_sim/camera_sim_component.hpp>

#include <rclcpp_components/register_node_macro.hpp>

namespace navi_sim
{
CameraSimComponent::CameraSimComponent(const rclcpp::NodeOptions & options)
: Node("camera_sim", options)
{
  initialize();
}

CameraSimComponent::CameraSimComponent(std::string name, const rclcpp::NodeOptions & options)
: Node(name, options)
{
  initialize();
}

void CameraSimComponent::initialize()
{
  double vertical_fov;
  declare_parameter("vertical_fov", 120.19512195121952);
  get_parameter("vertical_fov", vertical_fov);
  int horizontal_pixels;
  declare_parameter("horizontal_pixels", 720);
  get_parameter("horizontal_pixels", horizontal_pixels);
  int vertical_pixels;
  declare_parameter("vertical_pixels", 540);
  get_parameter("vertical_pixels", vertical_pixels);
  camera_info_ = sensor_msgs::msg::CameraInfo();
  camera_info_.height = vertical_pixels;
  camera_info_.width = vertical_pixels;
  camera_info_.distortion_model = "plumb_bob";
  camera_info_.d = {0, 0, 0, 0, 0};
  double f = static_cast<double>(vertical_pixels) * 0.5 /
    std::tan(vertical_fov * 0.5 / 180.0 * M_PI);
  camera_info_.k =
  {
    f, 0, static_cast<double>(horizontal_pixels) * 0.5,
    0, f, static_cast<double>(vertical_pixels) * 0.5,
    0, 0, 1
  };
  camera_info_.r =
  {
    1, 0, 0,
    0, 1, 0,
    0, 0, 1
  };
  camera_info_.p =
  {
    f, 0, static_cast<double>(horizontal_pixels) * 0.5, 0,
    0, f, static_cast<double>(vertical_pixels) * 0.5, 0,
    0, 0, 1, 0
  };
  cam_model_.fromCameraInfo(camera_info_);
  vision_info_pub_ = create_publisher<vision_msgs::msg::VisionInfo>("vision_info", 1);
  detection_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>("detection", 1);
}
}  // namespace navi_sim

RCLCPP_COMPONENTS_REGISTER_NODE(navi_sim::CameraSimComponent)
