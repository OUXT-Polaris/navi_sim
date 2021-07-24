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

#include <navi_sim/interpreter/action_base.hpp>
#include <string>
#include <vector>
#include <iostream>

namespace navi_sim
{
namespace actions
{
ActionType toActionTypeEnum(const std::string string_val)
{
  if (string_val == "send_goal") {
    return ActionType::SEND_GOAL;
  } else {
    throw std::runtime_error("invalid action type : " + string_val);
  }
}

std::string toActionTypeString(const ActionType & enum_val)
{
  std::string ret;
  switch (enum_val) {
    case ActionType::SEND_GOAL:
      ret = "send_goal";
      break;
  }
  return ret;
}

std::string toActionSateString(const ActionState & enum_val)
{
  std::string ret;
  switch (enum_val) {
    case ActionState::INACTIVE:
      ret = "inactive";
      break;
    case ActionState::ACTIVE:
      ret = "active";
      break;
    case ActionState::FINISHED:
      ret = "finished";
      break;
  }
  return ret;
}

ActionBase::ActionBase(const std::string & name, const YAML::Node & yaml)
: name(name), type(toActionTypeEnum(yaml["type"].as<std::string>()))
{
  state_ = ActionState::INACTIVE;
}

void ActionBase::update(const BlackBoard & black_board)
{
  const auto triggerd_actions = black_board.get<std::vector<std::string>>("triggerd_actions");
  switch (state_) {
    case ActionState::ACTIVE:
      {
        const auto state = onUpdate(black_board);
        state_ = state;
        break;
      }
    case ActionState::INACTIVE:
      {
        if (std::find(
            triggerd_actions.begin(),
            triggerd_actions.end(), name) != triggerd_actions.end())
        {
          state_ = ActionState::ACTIVE;
          update(black_board);
        }
        break;
      }
    case ActionState::FINISHED:
      {
        break;
      }
  }
}

ActionState ActionBase::getState() const
{
  return state_;
}

void ActionBase::getDebugString(YAML::Node & yaml)
{
  yaml["actions"][name]["type"] = toActionTypeString(type);
  yaml["actions"][name]["state"] = toActionSateString(state_);
}
}  // namespace actions
}  // namespace navi_sim
