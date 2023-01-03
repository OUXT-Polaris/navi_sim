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

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <boost/filesystem.hpp>
#include <fstream>
#include <memory>
#include <navi_sim/lidar_sim_component.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <string>
#include <vector>

namespace navi_sim
{
LidarSimComponent::LidarSimComponent(std::string name, const rclcpp::NodeOptions & options)
: Node(name, options), buffer_(get_clock()), listener_(buffer_)
{
  setParameters();
}

LidarSimComponent::LidarSimComponent(const rclcpp::NodeOptions & options)
: Node("lidar_sim", options), buffer_(get_clock()), listener_(buffer_, true)
{
  setParameters();
}

void LidarSimComponent::setParameters()
{
  using namespace std::chrono_literals;

  declare_parameter<std::string>("lidar_frame", "base_link");
  get_parameter("lidar_frame", lidar_frame_);
  std::cout << "frame : " << lidar_frame_ << std::endl;
  declare_parameter<std::string>("map_frame", "map");
  get_parameter("map_frame", map_frame_);
  declare_parameter<std::string>("embree_config", "threads=1,isa=avx");
  if (has_parameter("embree_config")) {
    std::string embree_config;
    get_parameter("embree_config", embree_config);
    raycaster_ptr_ = std::make_unique<Raycaster>(embree_config);
  } else {
    raycaster_ptr_ = std::make_unique<Raycaster>();
  }
  declare_parameter<std::string>("objects_filename", "objects.json");
  std::string objects_filename;
  get_parameter("objects_filename", objects_filename);
  objects_path_ =
    ament_index_cpp::get_package_share_directory("navi_sim") + "/config/" + objects_filename;
  declare_parameter<std::vector<double>>("vertical_angles", std::vector<double>(0));
  if (!has_parameter("vertical_angles")) {
    throw std::runtime_error("vertical_angles parameter does not exist");
  }
  vertical_angles_ = get_parameter("vertical_angles").as_double_array();
  declare_parameter<double>("max_distance", 100.0);
  get_parameter("max_distance", max_distance_);
  declare_parameter<double>("min_distance", 0.0);
  get_parameter("min_distance", min_distance_);
  declare_parameter<double>("start_horizontal_angle", 0.0);
  get_parameter("start_horizontal_angle", start_horizontal_angle_);
  declare_parameter<double>("end_horizontal_angle", M_PI * 2);
  get_parameter("end_horizontal_angle", end_horizontal_angle_);
  declare_parameter<double>("horizontal_resolution", 2 * M_PI / 360.0);
  get_parameter("horizontal_resolution", horizontal_resolution_);
  declare_parameter<double>("noise_distribution", 0.15);
  get_parameter("noise_distribution", noise_distribution_);
  declare_parameter<double>("double ghost_ratio", 0.001);
  get_parameter("ghost_ratio", ghost_ratio_);
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
    geometry_msgs::msg::TransformStamped transform_stamped =
      buffer_.lookupTransform(map_frame_, lidar_frame_, rclcpp::Time(0), tf2::durationFromSec(1.0));
    geometry_msgs::msg::Pose pose;
    pose.position.x = transform_stamped.transform.translation.x;
    pose.position.y = transform_stamped.transform.translation.y;
    pose.position.z = transform_stamped.transform.translation.z;
    pose.orientation = transform_stamped.transform.rotation;
    auto pointcloud_msg = raycaster_ptr_->raycast(
      pose, horizontal_resolution_, vertical_angles_, start_horizontal_angle_,
      end_horizontal_angle_, max_distance_, min_distance_, noise_distribution_);
    pointcloud_msg.header.stamp = now;
    pointcloud_msg.header.frame_id = lidar_frame_;
    pointcloud_pub_->publish(pointcloud_msg);
  } catch (tf2::ExtrapolationException & ex) {
    RCLCPP_ERROR(get_logger(), ex.what());
  }
}
}  // namespace navi_sim

RCLCPP_COMPONENTS_REGISTER_NODE(navi_sim::LidarSimComponent)
