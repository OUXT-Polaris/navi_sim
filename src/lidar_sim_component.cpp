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

#include <nlohmann/json.hpp>

#include <boost/filesystem.hpp>

#include <string>
#include <fstream>
#include <memory>

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
  declare_parameter("embree_config");
  if (has_parameter("embree_config")) {
    std::string embree_config;
    get_parameter("embree_config", embree_config);
    raycaster_ptr_ = std::make_unique<Raycaster>(embree_config);
  } else {
    raycaster_ptr_ = std::make_unique<Raycaster>();
  }
  declare_parameter("objects_path");
  if (!has_parameter("objects_path")) {
    throw std::runtime_error("objects_path parameter does not exist");
  }
  get_parameter("objects_path", objects_path_);
  namespace fs = boost::filesystem;
  const fs::path path(objects_path_);
  boost::system::error_code error;
  const bool result = fs::exists(path, error);
  if (!result || error) {
    throw std::runtime_error(objects_path_ + " did not find");
  }
  std::ifstream fin;
  fin.open(objects_path_, std::ios::in);
  std::string json_string = "";
  std::string line;
  while (std::getline(fin, line)) {
    json_string = json_string + line;
  }
  raycaster_ptr_->addPrimitives(nlohmann::json::parse(json_string));
  pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("lidar_points", 1);
  update_scan_timer_ =
    this->create_wall_timer(100ms, std::bind(&LidarSimComponent::updateScan, this));
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
    auto pointcloud_msg = raycaster_ptr_->raycast(
      pose, 2 * M_PI / 360.0,
      {
        DEG2RAD(-15.0), DEG2RAD(-13.0), DEG2RAD(-11.0), DEG2RAD(-9.0),
        DEG2RAD(-7.0), DEG2RAD(-5.0), DEG2RAD(-3.0), DEG2RAD(-1.0),
        DEG2RAD(1.0), DEG2RAD(3.0), DEG2RAD(5.0), DEG2RAD(7.0),
        DEG2RAD(9.0), DEG2RAD(11.0), DEG2RAD(13.0), DEG2RAD(15.0)
      });
    pointcloud_msg.header.stamp = now;
    pointcloud_msg.header.frame_id = lidar_frame_;
    pointcloud_pub_->publish(pointcloud_msg);
  } catch (tf2::ExtrapolationException & ex) {
    RCLCPP_ERROR(get_logger(), ex.what());
  }
}
}  // namespace navi_sim
