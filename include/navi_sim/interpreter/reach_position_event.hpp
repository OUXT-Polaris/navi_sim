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

#ifndef NAVI_SIM__INTERPRETER__REACH_POSITION_EVENT_HPP_
#define NAVI_SIM__INTERPRETER__REACH_POSITION_EVENT_HPP_

#include <yaml-cpp/yaml.h>
#include <navi_sim/interpreter/event_base.hpp>

#include <string>

namespace navi_sim
{
namespace events
{
class ReachPositionEvent : public EventBase
{
public:
  explicit ReachPositionEvent(const std::string & name, const YAML::Node & yaml);
  void getDebugString(YAML::Node & yaml) override;
};
}  // namespace events
}  // namespace navi_sim

#endif  // NAVI_SIM__INTERPRETER__REACH_POSITION_EVENT_HPP_
