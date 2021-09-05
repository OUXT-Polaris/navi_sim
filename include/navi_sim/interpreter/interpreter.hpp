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

#ifndef NAVI_SIM__INTERPRETER__INTERPRETER_HPP_
#define NAVI_SIM__INTERPRETER__INTERPRETER_HPP_

#include <navi_sim/interpreter/reach_position_event.hpp>
#include <navi_sim/interpreter/simulation_time_event.hpp>
#include <navi_sim/interpreter/send_goal_action.hpp>
#include <navi_sim/interpreter/terminate_action.hpp>

#include <yaml-cpp/yaml.h>
#include <string>
#include <memory>
#include <vector>
#include <utility>

namespace navi_sim
{
class Interpreter
{
public:
  Interpreter() = delete;
  explicit Interpreter(const std::string & path);
  size_t getEventIndex(const std::string & name) const;
  size_t getActionIndex(const std::string & name) const;
  actions::ActionState getActionState(const std::string & name) const;
  void getDebugString(YAML::Node & yaml);
  template<typename T>
  void setValueToBlackBoard(const std::string & key, const T & value)
  {
    black_board_.set(key, value);
  }
  void evaluate();

private:
  const YAML::Node appendDefaultActions(const YAML::Node & scenario);
  BlackBoard black_board_;
  const YAML::Node scenario_;
  std::vector<std::unique_ptr<events::EventBase>> events_;
  std::vector<std::unique_ptr<actions::ActionBase>> actions_;
  template<typename T>
  void addEvent(const std::string & name, const YAML::Node & yaml)
  {
    events_.emplace_back(std::make_unique<T>(name, yaml));
  }
  template<typename T>
  void addAction(const std::string & name, const YAML::Node & yaml)
  {
    actions_.emplace_back(std::make_unique<T>(name, yaml));
  }
};
}  // namespace navi_sim

#endif  // NAVI_SIM__INTERPRETER__INTERPRETER_HPP_
