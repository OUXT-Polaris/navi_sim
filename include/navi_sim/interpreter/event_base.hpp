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

#ifndef NAVI_SIM__INTERPRETER__EVENT_BASE_HPP_
#define NAVI_SIM__INTERPRETER__EVENT_BASE_HPP_

#include <navi_sim/interpreter/black_board.hpp>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <yaml-cpp/yaml.h>
#include <string>
#include <functional>

namespace navi_sim
{
namespace events
{
enum class EventState
{
  INACTIVE = 0,
  ACTIVE = 1,
  FINISHED = 2
};

enum class EventType
{
  REACH_POSITION = 0,
  SIMULATION_TIME = 1
};

void toEnum(const std::string string_val, EventType & enum_val);
EventType toEventTypeEnum(const std::string string_val);
std::string toEventTypeString(const EventType & enum_val);
std::string toEventStateString(const EventState & enum_val);

class EventBase
{
public:
  explicit EventBase(const std::string & name, const YAML::Node & yaml);
  const std::string name;
  const std::string trigger;
  const std::string next_action;
  const EventType type;
  void updateState(const BlackBoard & black_board);
  EventState getState() const;
  virtual void getDebugString(YAML::Node & yaml);

private:
  virtual EventState onUpdate(const BlackBoard & black_board) = 0;
  std::function<EventState(void)> func_;
  EventState state_;
};
}  // namespace events
}  // namespace navi_sim

#endif  // NAVI_SIM__INTERPRETER__EVENT_BASE_HPP_
