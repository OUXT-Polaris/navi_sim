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

#include <navi_sim/interpreter/reach_position_event.hpp>
#include <navi_sim/interpreter/data_type.hpp>

#include <string>
#include <iostream>
#include <cmath>

namespace navi_sim
{
namespace events
{
ReachPositionEvent::ReachPositionEvent(const std::string & name, const YAML::Node & yaml)
: EventBase(name, yaml)
{
  parse(yaml["target"], goal_);
  radius_ = yaml["radius"].as<double>();
}

void ReachPositionEvent::getDebugString(YAML::Node & yaml)
{
  EventBase::getDebugString(yaml);
}

EventState ReachPositionEvent::onUpdate(const BlackBoard & black_board)
{
  const auto ego_pose = black_board.get<geometry_msgs::msg::Pose>("ego_pose");
  double distance =
    std::sqrt(
    std::pow(ego_pose.position.x - goal_.pose.position.x, 2) +
    std::pow(ego_pose.position.y - goal_.pose.position.y, 2) +
    std::pow(ego_pose.position.z - goal_.pose.position.z, 2) );
  if (distance <= radius_) {
    return EventState::FINISHED;
  }
  return EventState::ACTIVE;
}
}  // namespace events
}  // namespace navi_sim
