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
}  // namespace navi_sim
