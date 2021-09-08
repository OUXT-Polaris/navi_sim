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

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <navi_sim/interpreter/data_type.hpp>
#include <navi_sim/interpreter/send_goal_action.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>

namespace navi_sim
{
namespace actions
{
SendGoalAction::SendGoalAction(const std::string & name, const YAML::Node & yaml)
: ActionBase(name, yaml)
{
  parse(yaml["goal"], goal_);
}

void SendGoalAction::getDebugString(YAML::Node & yaml) { ActionBase::getDebugString(yaml); }

ActionState SendGoalAction::onUpdate(const BlackBoard & black_board)
{
  auto pub = black_board.get<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr>(
    "goal_publisher");
  auto clock = black_board.get<rclcpp::Clock::SharedPtr>("clock");
  if (pub == nullptr) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("interpreter"), "goal publisher is uninitialized.");
  }
  if (clock == nullptr) {
    RCLCPP_ERROR_STREAM(rclcpp::get_logger("interpreter"), "clock is uninitialized.");
  } else {
    goal_.header.stamp = clock->now();
    pub->publish(goal_);
    RCLCPP_INFO_STREAM(rclcpp::get_logger("interpreter"), "goal published.");
  }
  return ActionState::FINISHED;
}
}  // namespace actions
}  // namespace navi_sim
