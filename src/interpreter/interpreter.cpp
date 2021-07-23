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

#include <navi_sim/interpreter/interpreter.hpp>
#include <navi_sim/interpreter/reach_position_event.hpp>

#include <fstream>
#include <string>
#include <iostream>

namespace navi_sim
{
Interpreter::Interpreter(const std::string & path)
: scenario_(YAML::LoadFile(path))
{
  const auto event_tree = scenario_["scenario"]["events"];
  for (YAML::const_iterator it = event_tree.begin(); it != event_tree.end(); it++) {
    const std::string name = it->first.as<std::string>();
    const auto event = event_tree[name];
    switch (events::toEventTypeEnum(event["type"].as<std::string>())) {
      case events::EventType::REACH_POSITION:
        addEvent<events::ReachPositionEvent>(name, event);
        break;
    }
  }
}

void Interpreter::getDebugString(YAML::Node & yaml)
{
  for (const auto & event : events_) {
    event->getDebugString(yaml);
  }
}

size_t Interpreter::getEventIndex(const std::string & name) const
{
  size_t index = 0;
  for (const auto & event : events_) {
    if (event->name == name) {
      return index;
    }
    index++;
  }
  throw std::runtime_error("event : " + name + " does not exist.");
}
}  // namespace navi_sim