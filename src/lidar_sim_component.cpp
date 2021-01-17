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

#include <navi_sim/lidar_sim_component.hpp>

#include <string>

namespace navi_sim
{
LidarSimComponent::LidarSimComponent(const rclcpp::NodeOptions & options)
: Node("navi_sim", options), buffer_(get_clock()), listener_(buffer_)
{
  using namespace std::chrono_literals;

  declare_parameter("lidar_frame", "base_link");
  get_parameter("lidar_frame", lidar_frame_);
  declare_parameter("map_frame", "map");
  get_parameter("map_frame", map_frame_);
  pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("lidar_points", 1);
  update_scan_timer_ =
    this->create_wall_timer(100ms, std::bind(&LidarSimComponent::updateScan, this));
}

void LidarSimComponent::addObject(
  std::string name, geometry_msgs::msg::Pose pose,
  std::string model)
{
  navi_sim::Mesh mesh = models.load(model);
  raycaster_.addObject(name, pose, mesh);
}

void LidarSimComponent::updateScan()
{
  rclcpp::Time now = get_clock()->now();
  try {
    geometry_msgs::msg::TransformStamped transform_stamped = buffer_.lookupTransform(
      map_frame_, lidar_frame_, now, tf2::durationFromSec(1.0));
    geometry_msgs::msg::Pose pose;
    pose.position.x = transform_stamped.transform.translation.x;
    pose.position.y = transform_stamped.transform.translation.y;
    pose.position.z = transform_stamped.transform.translation.z;
    pose.orientation = transform_stamped.transform.rotation;
    auto pointcloud_msg = raycaster_.raycast(
      pose, 2 * M_PI / 360.0,
      {
        RAD2DEG(-15.0), RAD2DEG(-13.0), RAD2DEG(-11.0), RAD2DEG(-9.0),
        RAD2DEG(-7.0), RAD2DEG(-5.0), RAD2DEG(-3.0), RAD2DEG(-1.0),
        RAD2DEG(1.0), RAD2DEG(3.0), RAD2DEG(5.0), RAD2DEG(7.0),
        RAD2DEG(9.0), RAD2DEG(11.0), RAD2DEG(13.0), RAD2DEG(15.0)
      });
    pointcloud_msg.header.stamp = now;
    pointcloud_msg.header.frame_id = lidar_frame_;
    pointcloud_pub_->publish(pointcloud_msg);
  } catch (tf2::ExtrapolationException & ex) {
    RCLCPP_ERROR(get_logger(), ex.what());
  }
}
}  // namespace navi_sim
