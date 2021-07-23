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

#ifndef NAVI_SIM__INTERPRETER__SEND_GOAL_ACTION_HPP_
#define NAVI_SIM__INTERPRETER__SEND_GOAL_ACTION_HPP_

#include <navi_sim/interpreter/action_base.hpp>
#include <navi_sim/interpreter/black_board.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <string>

namespace navi_sim
{
namespace actions
{
class SendGoalAction : public ActionBase
{
public:
  explicit SendGoalAction(const std::string & name, const YAML::Node & yaml);
  void getDebugString(YAML::Node & yaml) override;

private:
  geometry_msgs::msg::PoseStamped goal_;
  ActionState onUpdate(const BlackBoard & black_board) override;
};
}  // namespace actions
}  // namespace navi_sim

#endif  // NAVI_SIM__INTERPRETER__SEND_GOAL_ACTION_HPP_
