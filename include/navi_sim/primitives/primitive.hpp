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

#ifndef NAVI_SIM__PRIMITIVES__PRIMITIVE_HPP_
#define NAVI_SIM__PRIMITIVES__PRIMITIVE_HPP_

#include <geometry_msgs/msg/pose.hpp>

#include <string>
#include <vector>

namespace navi_sim
{
struct Vertex
{
  float x;
  float y;
  float z;
};

struct Triangle
{
  int v0;
  int v1;
  int v2;
};

class Primitive
{
public:
  Primitive(std::string type, geometry_msgs::msg::Pose pose);
  const std::string type;
  const geometry_msgs::msg::Pose pose;

private:
  std::vector<Vertex> vertex_;
  std::vector<Triangle> triangles_;

protected:
  Vertex transform(Vertex v);
};
}  // namespace navi_sim

#endif  // NAVI_SIM__PRIMITIVES__PRIMITIVE_HPP_