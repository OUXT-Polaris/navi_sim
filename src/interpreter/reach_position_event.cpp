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

#include <navi_sim/interpreter/reach_position_event.hpp>

namespace navi_sim
{
namespace events
{
ReachPositionEvent::ReachPositionEvent(
  const std::string & name,
  const std::string & trigger,
  const std::string & next_action)
: EventBase(name, trigger, next_action, EventType::REACH_POSITION)
{

}
}  // namespace events
}  // namespace navi_sim
