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

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include <fstream>
#include <string>
#include <iostream>
#include <set>
#include <vector>

namespace navi_sim
{
Interpreter::Interpreter(const std::string & path)
: scenario_(YAML::LoadFile(path))
{
  const auto scenario = appendDefaultActions(scenario_);
  const auto event_tree = scenario["scenario"]["events"];
  for (YAML::const_iterator it = event_tree.begin(); it != event_tree.end(); it++) {
    const std::string name = it->first.as<std::string>();
    const auto event = event_tree[name];
    switch (events::toEventTypeEnum(event["type"].as<std::string>())) {
      case events::EventType::REACH_POSITION:
        addEvent<events::ReachPositionEvent>(name, event);
        break;
      case events::EventType::SIMULATION_TIME:
        addEvent<events::SimulationTimeEvent>(name, event);
        break;
    }
  }
  const auto action_tree = scenario["scenario"]["actions"];
  for (YAML::const_iterator it = action_tree.begin(); it != action_tree.end(); it++) {
    const std::string name = it->first.as<std::string>();
    const auto action = action_tree[name];
    switch (actions::toActionTypeEnum(action["type"].as<std::string>())) {
      case actions::ActionType::SEND_GOAL:
        addAction<actions::SendGoalAction>(name, action);
        break;
      case actions::ActionType::TERMINATE:
        addAction<actions::TerminateAction>(name, action);
        break;
    }
  }
  black_board_.set<rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr>(
    "goal_publisher",
    nullptr);
  black_board_.set<double>("simulation_time", 0);
  black_board_.set<rclcpp::Clock::SharedPtr>("clock", nullptr);
  black_board_.set<std::vector<std::string>>("activated_events", {});
  black_board_.set<std::vector<std::string>>("triggerd_actions", {});
}

const YAML::Node Interpreter::appendDefaultActions(const YAML::Node & scenario)
{
  YAML::Node node;
  node = scenario;
  node["scenario"]["actions"]["success"]["type"] = "terminate";
  node["scenario"]["actions"]["success"]["success"] = true;
  node["scenario"]["actions"]["failure"]["type"] = "terminate";
  node["scenario"]["actions"]["failure"]["success"] = true;
  return node;
}

void Interpreter::evaluate()
{
  std::vector<std::string> activated_events;
  std::vector<std::string> triggerd_actions;
  for (const auto & event : events_) {
    if (event->getState() != events::EventState::INACTIVE) {
      activated_events.emplace_back(event->name);
    }
  }
  black_board_.set("activated_events", activated_events);
  for (const auto & event : events_) {
    event->updateState(black_board_);
    if (event->getState() == events::EventState::FINISHED) {
      triggerd_actions.emplace_back(event->next_action);
    }
  }
  black_board_.set("triggerd_actions", triggerd_actions);
  for (const auto & action : actions_) {
    action->update(black_board_);
  }
}

void Interpreter::getDebugString(YAML::Node & yaml)
{
  for (const auto & action : actions_) {
    action->getDebugString(yaml);
  }
  for (const auto & event : events_) {
    event->getDebugString(yaml);
  }
}

actions::ActionState Interpreter::getActionState(const std::string & name) const
{
  return actions_[getActionIndex(name)]->getState();
}

size_t Interpreter::getActionIndex(const std::string & name) const
{
  size_t index = 0;
  for (const auto & action : actions_) {
    if (action->name == name) {
      return index;
    }
    index++;
  }
  throw std::runtime_error("action : " + name + " does not exist.");
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
