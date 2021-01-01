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

struct Vertex
{
  std::array<float, 3> m_pos;
  std::array<float, 2> m_tex;
  std::array<float, 3> m_normal;

  Vertex() {}

  Vertex(
    const std::array<float, 3> & pos,
    const std::array<float, 2> & tex,
    const std::array<float, 3> & normal)
  {
    m_pos = pos;
    m_tex = tex;
    m_normal = normal;
  }
};

#endif  // NAVI_SIM__VERTEX_HPP_
