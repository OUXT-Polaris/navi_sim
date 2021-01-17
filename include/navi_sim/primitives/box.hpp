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

#include <navi_sim/mesh.hpp>
#include <navi_sim/vertex.hpp>

#include <array>
#include <vector>

namespace navi_sim
{
class Box
{
public:
  Box(double deapth, double width, double height);
  Mesh mesh();

private:
  std::vector<navi_sim::Vertex> vertices_;
  std::vector<std::array<unsigned int, 3>> indices_;
};
}  // namespace navi_sim

#endif  // NAVI_SIM__PRIMITIVES__BOX_HPP_
