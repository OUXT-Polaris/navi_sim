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

#include <navi_sim/interpreter/black_board.hpp>
#include <string>

namespace navi_sim
{
boost::any BlackBoard::getValue(const std::string & key) const
{
  if (data_.find(key) == data_.end()) {
    throw std::runtime_error("key : " + key + " does not exist.");
  }
  return data_.at(key);
}
}  // namespace navi_sim
