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

}
}  // namespace navi_sim
