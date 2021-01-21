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

#include <navi_sim/primitives/box.hpp>

namespace navi_sim
{
Box::Box(double depth, double width, double height, geometry_msgs::msg::Pose pose)
: Primitive("Box", pose)
{
  vertices_ = std::vector<Vertex>(8);
  vertices_[0].x = -0.5 * depth;
  vertices_[0].y = -0.5 * width;
  vertices_[0].z = -0.5 * height;
  vertices_[1].x = -0.5 * depth;
  vertices_[1].y = -0.5 * width;
  vertices_[1].z = +0.5 * height;
  vertices_[2].x = -0.5 * depth;
  vertices_[2].y = +0.5 * width;
  vertices_[2].z = -0.5 * height;
  vertices_[3].x = -0.5 * depth;
  vertices_[3].y = +0.5 * width;
  vertices_[3].z = +0.5 * height;
  vertices_[4].x = +0.5 * depth;
  vertices_[4].y = -0.5 * width;
  vertices_[4].z = -0.5 * height;
  vertices_[5].x = +0.5 * depth;
  vertices_[5].y = -0.5 * width;
  vertices_[5].z = +0.5 * height;
  vertices_[6].x = +0.5 * depth;
  vertices_[6].y = +0.5 * width;
  vertices_[6].z = -0.5 * height;
  vertices_[7].x = +0.5 * depth;
  vertices_[7].y = +0.5 * width;
  vertices_[7].z = +0.5 * height;
  transform();
}
}  // namespace navi_sim
