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

#ifndef NAVI_SIM__PRIMITIVES__BOX_HPP_
#define NAVI_SIM__PRIMITIVES__BOX_HPP_

#include <navi_sim/primitives/primitive.hpp>
#include <nlohmann/json.hpp>

namespace navi_sim
{
class Box : public Primitive
{
public:
  explicit Box(float depth, float width, float height, geometry_msgs::msg::Pose pose);
  Box();
  ~Box() = default;
  const float depth;
  const float width;
  const float height;
  Box & operator=(const Box &)
  {
    return *this;
  }
  nlohmann::json toJson() const;
};

void to_json(nlohmann::json & j, const Box & p);
void from_json(const nlohmann::json & j, Box & p);
}  // namespace navi_sim

#endif  // NAVI_SIM__PRIMITIVES__BOX_HPP_
