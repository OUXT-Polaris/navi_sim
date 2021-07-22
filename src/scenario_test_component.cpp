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

#include <string>

namespace navi_sim
{
ScenarioTestComponent::ScenarioTestComponent(const rclcpp::NodeOptions & options)
: Node("camera_sim", options)
{
  initialize();
}

ScenarioTestComponent::ScenarioTestComponent(std::string name, const rclcpp::NodeOptions & options)
: Node(name, options)
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
  declare_parameter("bbox_height", 0.0);
  get_parameter("bbox_height", bbox_height_);

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
  raycaster_ptr_->addPrimitives(nlohmann::json::parse(json_string));
}

bool ScenarioTestComponent::checkCollision()
{
  for(const auto & name : raycaster_ptr_->getPrimitiveNames()) {
    raycaster_ptr_->get2DPolygon(name);
  }
}

bool ScenarioTestComponent::checkCollision(
  const std::vector<geometry_msgs::msg::Point> & poly0,
  const std::vector<geometry_msgs::msg::Point> & poly1)
{
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
  return true;
}
}  // namespace navi_sim

RCLCPP_COMPONENTS_REGISTER_NODE(navi_sim::ScenarioTestComponent)
