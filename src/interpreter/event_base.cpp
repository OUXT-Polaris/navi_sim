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

#include <navi_sim/interpreter/event_base.hpp>

#include <fstream>
#include <string>

namespace navi_sim
{
namespace events
{
void toEnum(const std::string string_val, EventType & enum_val)
{
  enum_val = toEventTypeEnum(string_val);
}

EventType toEventTypeEnum(const std::string string_val)
{
  if (string_val == "reach_position") {
    return EventType::REACH_POSITION;
  } else {
    throw std::runtime_error("invalid event name : " + string_val);
  }
}

std::string toEventTypeString(const EventType & enum_val)
{
  std::string ret;
  switch (enum_val)
  {
  case EventType::REACH_POSITION:
    ret = "reach_position";
    break;
  }
  return ret;
}

std::string toEventStateString(const EventState & enum_val)
{
  std::string ret;
  switch (enum_val)
  {
  case EventState::INACTIVE:
    ret = "inactive";
    break;
  case EventState::ACTIVE:
    ret = "active";
    break;
  case EventState::FINISHED:
    ret = "finished";
    break;
  }
  return ret;
}

EventBase::EventBase(const std::string & name, const YAML::Node & yaml)
: name(name),
  trigger(yaml["trigger"].as<std::string>()),
  next_action(yaml["next_action"].as<std::string>()),
  type(toEventTypeEnum(yaml["type"].as<std::string>()))
{
  state_ = EventState::INACTIVE;
}

void EventBase::registerFunction(const std::function<EventState(void)> & func)
{
  func_ = func;
}

void EventBase::activate()
{
  state_ = EventState::ACTIVE;
}

EventState EventBase::getState() const
{
  return state_;
}

void EventBase::getDebugString(YAML::Node & yaml)
{
  yaml["events"][name]["type"] = toEventTypeString(type);
  yaml["events"][name]["state"] = toEventStateString(state_);
}

void EventBase::update()
{
  if (!func_) {
    throw std::runtime_error("function does not registered.");
  }
  switch (state_) {
    case EventState::ACTIVE:
      {
        const auto state = func_();
        state_ = state;
        break;
      }
    default:
      {
        break;
      }
  }
}
}  // namespace events
}  // namespace navi_sim
