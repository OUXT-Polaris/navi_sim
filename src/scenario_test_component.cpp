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

#include <navi_sim/scenario_test_component.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp_components/register_node_macro.hpp>

#include <boost/assert.hpp>
#include <boost/assign/list_of.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/algorithms/disjoint.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>

#include <quaternion_operation/quaternion_operation.h>

#include <fstream>
#include <string>
#include <vector>
#include <memory>

namespace navi_sim
{
ScenarioTestComponent::ScenarioTestComponent(const rclcpp::NodeOptions & options)
: Node("scenario_test", options), buffer_(get_clock()), listener_(buffer_)
{
  initialize();
}

ScenarioTestComponent::ScenarioTestComponent(std::string name, const rclcpp::NodeOptions & options)
: Node(name, options), buffer_(get_clock()), listener_(buffer_)
{
  initialize();
}

void ScenarioTestComponent::initialize()
{
  declare_parameter("bbox_center_x", 0.0);
  get_parameter("bbox_center_x", bbox_center_x_);
  declare_parameter("bbox_center_y", 0.0);
  get_parameter("bbox_center_y", bbox_center_y_);
  declare_parameter("bbox_center_z", 0.0);
  get_parameter("bbox_center_z", bbox_center_z_);
  declare_parameter("bbox_length", 0.0);
  get_parameter("bbox_length", bbox_length_);
  declare_parameter("bbox_width", 0.0);
  get_parameter("bbox_width", bbox_width_);
  declare_parameter("map_frame", "map");
  get_parameter("map_frame", map_frame_);
  declare_parameter("scenario_filename", "");
  get_parameter("scenario_filename", scenario_filename_);
  std::string scenario_path = ament_index_cpp::get_package_share_directory("navi_sim") +
    "/scenarios/" + scenario_filename_;
  declare_parameter("objects_filename", "objects.json");
  std::string objects_filename;
  get_parameter("objects_filename", objects_filename);
  std::string objects_path = ament_index_cpp::get_package_share_directory("navi_sim") + "/config/" +
    objects_filename;
  namespace fs = boost::filesystem;
  const fs::path path(objects_path);
  boost::system::error_code error;
  const bool result = fs::exists(path, error);
  if (!result || error) {
    throw std::runtime_error(objects_path + " did not find");
  }
  std::ifstream fin;
  fin.open(objects_path, std::ios::in);
  std::string json_string = "";
  std::string line;
  while (std::getline(fin, line)) {
    json_string = json_string + line;
  }
  raycaster_ptr_ = std::make_unique<Raycaster>();
  raycaster_ptr_->addPrimitives(nlohmann::json::parse(json_string));
  collision_marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
    "collision",
    1);
  interpreter_ = std::make_unique<navi_sim::Interpreter>(scenario_path);
  goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/move_base_simple/goal", 1);
  interpreter_->setValueToBlackBoard<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr>(
    "goal_publisher", goal_pub_);
  context_pub_ = this->create_publisher<std_msgs::msg::String>("context", 1);
  interpreter_->setValueToBlackBoard<rclcpp::Clock::SharedPtr>("clock", get_clock());
  start_time_ = get_clock()->now();
  using namespace std::chrono_literals;
  timer_ = create_wall_timer(50ms, std::bind(&ScenarioTestComponent::update, this));
}

const visualization_msgs::msg::MarkerArray ScenarioTestComponent::getCollisionMarker(bool collision)
{
  const auto stamp = get_clock()->now();
  visualization_msgs::msg::MarkerArray markers;
  const auto bbox_polygon = getBboxPolygon();
  if (bbox_polygon.empty()) {
    return markers;
  }
  std_msgs::msg::ColorRGBA color_ego_collision_marker;
  if (collision) {
    color_ego_collision_marker.r = 1.0;
    color_ego_collision_marker.g = 0.0;
    color_ego_collision_marker.b = 0.0;
    color_ego_collision_marker.a = 0.9999;
  } else {
    color_ego_collision_marker.r = 0.0;
    color_ego_collision_marker.g = 1.0;
    color_ego_collision_marker.b = 0.0;
    color_ego_collision_marker.a = 0.9999;
  }
  visualization_msgs::msg::Marker ego_collision_marker;
  ego_collision_marker.header.stamp = stamp;
  ego_collision_marker.header.frame_id = map_frame_;
  ego_collision_marker.id = 0;
  ego_collision_marker.ns = "ego_collision";
  ego_collision_marker.points = bbox_polygon;
  ego_collision_marker.points.emplace_back(bbox_polygon[0]);
  ego_collision_marker.scale.x = 0.03;
  ego_collision_marker.scale.y = 0.03;
  ego_collision_marker.scale.z = 0.03;
  ego_collision_marker.action = ego_collision_marker.ADD;
  ego_collision_marker.color = color_ego_collision_marker;
  ego_collision_marker.type = ego_collision_marker.LINE_STRIP;
  markers.markers.emplace_back(ego_collision_marker);
  size_t index = 0;
  for (const auto & name : raycaster_ptr_->getPrimitiveNames()) {
    visualization_msgs::msg::Marker marker;
    marker.header.stamp = stamp;
    marker.header.frame_id = map_frame_;
    marker.ns = "obstacle";
    marker.id = index;
    marker.points = raycaster_ptr_->get2DPolygon(name);
    if (marker.points.empty()) {
      continue;
    }
    marker.points.emplace_back(marker.points[0]);
    marker.scale.x = 0.03;
    marker.scale.y = 0.03;
    marker.scale.z = 0.03;
    marker.action = marker.ADD;
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.9999;
    marker.type = marker.LINE_STRIP;
    markers.markers.emplace_back(marker);
    index = index + 1;
  }
  return markers;
}

void ScenarioTestComponent::update()
{
  interpreter_->setValueToBlackBoard(
    "simulation_time",
    (get_clock()->now() - start_time_).seconds());
  interpreter_->setValueToBlackBoard("ego_pose", getEgoPose());
  interpreter_->evaluate();
  std::stringstream ss;
  YAML::Node context;
  interpreter_->getDebugString(context);
  ss << context;
  std_msgs::msg::String string_data;
  string_data.data = ss.str();
  context_pub_->publish(string_data);
  // std::cout << context << std::endl;
  const auto collision = checkCollision();
  collision_marker_pub_->publish(getCollisionMarker(collision));
  std::ofstream output("/tmp/context.yaml", std::ios::out | std::ios::trunc);
  output << ss.str();
  output.close();
  if (interpreter_->getActionState("success") == actions::ActionState::FINISHED ||
    interpreter_->getActionState("failure") == actions::ActionState::FINISHED)
  {
    rclcpp::shutdown();
  }
}

bool ScenarioTestComponent::checkCollision()
{
  const auto bbox_polygon = getBboxPolygon();
  if (bbox_polygon.empty()) {
    return false;
  }
  for (const auto & name : raycaster_ptr_->getPrimitiveNames()) {
    if (checkCollision(bbox_polygon, raycaster_ptr_->get2DPolygon(name))) {
      return true;
    }
  }
  return false;
}

const boost::optional<geometry_msgs::msg::Pose> ScenarioTestComponent::getEgoPose()
{
  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = buffer_.lookupTransform(
      map_frame_, "base_link", rclcpp::Time(0), tf2::durationFromSec(1.0));
    geometry_msgs::msg::Pose pose;
    pose.position.x = transform_stamped.transform.translation.x;
    pose.position.y = transform_stamped.transform.translation.y;
    pose.position.z = transform_stamped.transform.translation.z;
    pose.orientation = transform_stamped.transform.rotation;
    return pose;
  } catch (tf2::ExtrapolationException & ex) {
    RCLCPP_ERROR(get_logger(), ex.what());
    return boost::none;
  }
  return boost::none;
}

const std::vector<geometry_msgs::msg::Point> ScenarioTestComponent::getBboxPolygon()
{
  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = buffer_.lookupTransform(
      map_frame_, "base_link", rclcpp::Time(0), tf2::durationFromSec(1.0));
  } catch (tf2::ExtrapolationException & ex) {
    RCLCPP_ERROR(get_logger(), ex.what());
    return {};
  }
  std::vector<geometry_msgs::msg::Point> points;
  points.emplace_back(getCornerPoint(true, true));
  points.emplace_back(getCornerPoint(true, false));
  points.emplace_back(getCornerPoint(false, false));
  points.emplace_back(getCornerPoint(false, true));
  points.emplace_back(getCornerPoint(true, true));
  return transformPoints(transform_stamped, points);
}

const geometry_msgs::msg::Point ScenarioTestComponent::getCornerPoint(
  bool x_dir, bool y_dir) const
{
  geometry_msgs::msg::Point p;
  if (x_dir) {
    p.x = bbox_center_x_ + bbox_length_ * 0.5;
  } else {
    p.x = bbox_center_x_ - bbox_length_ * 0.5;
  }
  if (y_dir) {
    p.y = bbox_center_y_ + bbox_width_ * 0.5;
  } else {
    p.y = bbox_center_y_ - bbox_width_ * 0.5;
  }
  p.z = bbox_center_z_;
  return p;
}

std::vector<geometry_msgs::msg::Point> ScenarioTestComponent::transformPoints(
  const geometry_msgs::msg::TransformStamped & pose,
  const std::vector<geometry_msgs::msg::Point> & points)
{
  auto mat = quaternion_operation::getRotationMatrix(pose.transform.rotation);
  std::vector<geometry_msgs::msg::Point> ret;
  for (const auto & point : points) {
    Eigen::VectorXd v(3);
    v(0) = point.x;
    v(1) = point.y;
    v(2) = point.z;
    v = mat * v;
    v(0) = v(0) + pose.transform.translation.x;
    v(1) = v(1) + pose.transform.translation.y;
    v(2) = v(2) + pose.transform.translation.z;
    geometry_msgs::msg::Point transformed;
    transformed.x = v(0);
    transformed.y = v(1);
    transformed.z = v(2);
    ret.emplace_back(transformed);
  }
  return ret;
}

bool ScenarioTestComponent::checkCollision(
  const std::vector<geometry_msgs::msg::Point> & poly0,
  const std::vector<geometry_msgs::msg::Point> & poly1)
{
  if (poly0.empty() || poly1.empty()) {
    return false;
  }
  namespace bg = boost::geometry;
  typedef bg::model::d2::point_xy<double> bg_point;
  bg::model::polygon<bg_point> poly0_bg;
  for (const auto & p : poly0) {
    poly0_bg.outer().push_back(bg_point(p.x, p.y));
  }
  bg::model::polygon<bg_point> poly1_bg;
  for (const auto & p : poly1) {
    poly1_bg.outer().push_back(bg_point(p.x, p.y));
  }
  if (bg::intersects(poly0_bg, poly1_bg)) {
    return true;
  }
  if (bg::intersects(poly1_bg, poly0_bg)) {
    return true;
  }
  if (bg::disjoint(poly0_bg, poly1_bg)) {
    return false;
  }
  return false;
}
}  // namespace navi_sim

RCLCPP_COMPONENTS_REGISTER_NODE(navi_sim::ScenarioTestComponent)
