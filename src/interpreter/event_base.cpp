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
EventBase::EventBase(
  const std::string & name,
  const std::string & trigger,
  const std::string & next_action,
  const EventType & type)
: name(name),
  trigger(trigger),
  next_action(next_action),
  type(type)
{
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
