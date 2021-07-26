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

#include <navi_sim/interpreter/terminate_action.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace navi_sim
{
namespace actions
{
TerminateAction::TerminateAction(const std::string & name, const YAML::Node & yaml)
: ActionBase(name, yaml)
{
  success_ = yaml["success"].as<bool>();
}

void TerminateAction::getDebugString(YAML::Node & yaml)
{
  ActionBase::getDebugString(yaml);
}

ActionState TerminateAction::onUpdate(const BlackBoard &)
{
  rclcpp::shutdown();
  return ActionState::FINISHED;
}
}  // namespace actions
}  // namespace navi_sim