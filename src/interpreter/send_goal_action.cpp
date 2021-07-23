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

#include <navi_sim/interpreter/send_goal_action.hpp>
#include <navi_sim/interpreter/data_type.hpp>

namespace navi_sim
{
namespace actions
{
SendGoalAction::SendGoalAction(const std::string & name, const YAML::Node & yaml)
: ActionBase(name, yaml)
{
  parse(yaml["target"], goal_);
}
}  // namespace actions
}  // namespace navi_sim
