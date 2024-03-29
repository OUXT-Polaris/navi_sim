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
#include <navi_sim/semantic_map_sim_component.hpp>
#include <nlohmann/json.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <string>
#include <vector>

namespace navi_sim
{
SemanticMapSimComponent::SemanticMapSimComponent(
  std::string name, const rclcpp::NodeOptions & options)
: Node(name, options), buffer_(get_clock()), listener_(buffer_)
{
  setParameters();
}

SemanticMapSimComponent::SemanticMapSimComponent(const rclcpp::NodeOptions & options)
: Node("lidar_sim", options), buffer_(get_clock()), listener_(buffer_)
{
  setParameters();
}

void SemanticMapSimComponent::setParameters()
{
  using namespace std::chrono_literals;
  declare_parameter<std::string>("map_frame", "map");
  get_parameter("map_frame", map_frame_);
  declare_parameter<std::string>("robot_frame", "base_link");
  get_parameter("robot_frame", robot_frame_);
  declare_parameter<std::string>("task_objects_topic", "task_objects");
  get_parameter("task_objects_topic", task_objects_topic_);
  declare_parameter<std::string>("embree_config", "");
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
  declare_parameter<double>("detection_distance", 10.0);
  get_parameter<double>("detection_distance", detection_distance_);
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
  task_objects_pub_ = this->create_publisher<robotx_behavior_msgs::msg::TaskObjectsArrayStamped>(
    task_objects_topic_, 1);
  update_map_timer_ =
    this->create_wall_timer(100ms, std::bind(&SemanticMapSimComponent::updateMap, this));
}

void SemanticMapSimComponent::updateMap()
{
  rclcpp::Time now = get_clock()->now();
  try {
    geometry_msgs::msg::TransformStamped transform_stamped =
      buffer_.lookupTransform(map_frame_, robot_frame_, rclcpp::Time(0), tf2::durationFromSec(1.0));
    geometry_msgs::msg::Pose pose;
    pose.position.x = transform_stamped.transform.translation.x;
    pose.position.y = transform_stamped.transform.translation.y;
    pose.position.z = transform_stamped.transform.translation.z;
    pose.orientation = transform_stamped.transform.rotation;
    const auto detected_objects =
      raycaster_ptr_->queryByDistance(pose.position, detection_distance_);
    for (const auto & obj : detected_objects) {
      if (tracked_objects_.find(obj) == tracked_objects_.end()) {
        tracked_objects_[obj] = std::make_pair(tracked_objects_.size(), now);
      }
    }
    robotx_behavior_msgs::msg::TaskObjectsArrayStamped msg;
    msg.header.stamp = now;
    msg.header.frame_id = map_frame_;
    size_t unique_id = 0;
    for (const auto & obj : tracked_objects_) {
      auto task_object = raycaster_ptr_->getTaskObject(obj.first);
      if (obj.second.second == now) {
        task_object.detected = true;
      } else {
        task_object.detected = false;
      }
      task_object.last_detected = obj.second.second;
      task_object.unique_id = unique_id;
      msg.task_objects.emplace_back(task_object);
      ++unique_id;
    }
    task_objects_pub_->publish(msg);
  } catch (tf2::ExtrapolationException & ex) {
    RCLCPP_ERROR(get_logger(), ex.what());
  }
}
}  // namespace navi_sim

RCLCPP_COMPONENTS_REGISTER_NODE(navi_sim::SemanticMapSimComponent)
