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

#ifndef NAVI_SIM__VERTEX_HPP_
#define NAVI_SIM__VERTEX_HPP_

#include <array>
#include <geometry_msgs/msg/point.hpp>

namespace navi_sim
{
class Vertex
{
public:
  Vertex(
    geometry_msgs::msg::Point position,
    double texture_position_x,
    double texture_position_y);
  const geometry_msgs::msg::Point getPosition() const
  {
    return position_;
  }
  void setPosition(const geometry_msgs::msg::Point & p)
  {
    position_ = p;
  }

private:
  geometry_msgs::msg::Point position_;
  std::array<double, 2> texture_position_;
};
} // namespace navi_sim

#endif  // NAVI_SIM__VERTEX_HPP_
