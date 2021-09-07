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

#ifndef NAVI_SIM__INTERPRETER__ACTION_BASE_HPP_
#define NAVI_SIM__INTERPRETER__ACTION_BASE_HPP_

#include <yaml-cpp/yaml.h>

#include <navi_sim/interpreter/black_board.hpp>
#include <string>

namespace navi_sim
{
namespace actions
{
enum class ActionState { INACTIVE = 0, ACTIVE = 1, FINISHED = 2 };

enum class ActionType { SEND_GOAL = 0, TERMINATE = 1 };

ActionType toActionTypeEnum(const std::string string_val);
std::string toActionTypeString(const ActionType & enum_val);
std::string toActionSateString(const ActionState & enum_val);

class ActionBase
{
public:
  explicit ActionBase(const std::string & name, const YAML::Node & yaml);
  const std::string name;
  const ActionType type;
  virtual void getDebugString(YAML::Node & yaml);
  ActionState getState() const;
  void update(const BlackBoard & black_board);

private:
  virtual ActionState onUpdate(const BlackBoard & black_board) = 0;
  ActionState state_;
};
}  // namespace actions
}  // namespace navi_sim

#endif  // NAVI_SIM__INTERPRETER__ACTION_BASE_HPP_
