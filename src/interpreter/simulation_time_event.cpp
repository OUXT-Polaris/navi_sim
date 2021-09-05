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

#include <navi_sim/interpreter/simulation_time_event.hpp>

#include <rclcpp/rclcpp.hpp>
#include <string>

namespace navi_sim
{
namespace events
{
SimulationTimeEvent::SimulationTimeEvent(const std::string & name, const YAML::Node & yaml)
: EventBase(name, yaml)
{
  value_ = yaml["value"].as<double>();
  grater_ = yaml["grater"].as<bool>();
}

void SimulationTimeEvent::getDebugString(YAML::Node & yaml)
{
  EventBase::getDebugString(yaml);
  if (getState() == events::EventState::ACTIVE) {
    yaml["events"][name]["simulation_time"] = simulation_time_;
  }
}

EventState SimulationTimeEvent::onUpdate(const BlackBoard & black_board)
{
  simulation_time_ = black_board.get<double>("simulation_time");
  if (grater_ && value_ <= simulation_time_) {
    return EventState::FINISHED;
  }
  if (!grater_ && value_ >= simulation_time_) {
    return EventState::FINISHED;
  }
  return EventState::ACTIVE;
}
}  // namespace events
}  // namespace navi_sim
