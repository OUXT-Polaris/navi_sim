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
Box::Box(double deapth, double width, double height)
{
  geometry_msgs::msg::Point p0;
  p0.x = deapth * 0.5;
  p0.y = width * 0.5;
  p0.z = height * 0.5;
  navi_sim::Vertex v0 = navi_sim::Vertex(p0);
  vertices_.emplace_back(v0);
  geometry_msgs::msg::Point p1;
  p1.x = deapth * -0.5;
  p1.y = width * 0.5;
  p1.z = height * 0.5;
  navi_sim::Vertex v1 = navi_sim::Vertex(p1);
  vertices_.emplace_back(v1);
  geometry_msgs::msg::Point p2;
  p2.x = deapth * -0.5;
  p2.y = width * -0.5;
  p2.z = height * 0.5;
  navi_sim::Vertex v2 = navi_sim::Vertex(p2);
  vertices_.emplace_back(v2);
  geometry_msgs::msg::Point p3;
  p3.x = deapth * 0.5;
  p3.y = width * -0.5;
  p3.z = height * 0.5;
  navi_sim::Vertex v3 = navi_sim::Vertex(p3);
  vertices_.emplace_back(v3);
  geometry_msgs::msg::Point p4;
  p4.x = deapth * 0.5;
  p4.y = width * 0.5;
  p4.z = height * -0.5;
  navi_sim::Vertex v4 = navi_sim::Vertex(p4);
  vertices_.emplace_back(v4);
  geometry_msgs::msg::Point p5;
  p5.x = deapth * -0.5;
  p5.y = width * 0.5;
  p5.z = height * -0.5;
  navi_sim::Vertex v5 = navi_sim::Vertex(p5);
  vertices_.emplace_back(v5);
  geometry_msgs::msg::Point p6;
  p6.x = deapth * -0.5;
  p6.y = width * -0.5;
  p6.z = height * -0.5;
  navi_sim::Vertex v6 = navi_sim::Vertex(p6);
  vertices_.emplace_back(v6);
  geometry_msgs::msg::Point p7;
  p7.x = deapth * 0.5;
  p7.y = width * -0.5;
  p7.z = height * -0.5;
  navi_sim::Vertex v7 = navi_sim::Vertex(p7);
  vertices_.emplace_back(v7);
  indices_.push_back({0, 1, 2});
  indices_.push_back({0, 2, 3});
  indices_.push_back({1, 5, 6});
  indices_.push_back({1, 6, 2});
  indices_.push_back({2, 6, 7});
  indices_.push_back({2, 7, 3});
  indices_.push_back({3, 7, 4});
  indices_.push_back({3, 4, 0});
  indices_.push_back({0, 4, 5});
  indices_.push_back({0, 5, 1});
  indices_.push_back({4, 5, 6});
  indices_.push_back({4, 6, 7});
}

Mesh Box::mesh()
{
  return Mesh(vertices_, indices_);
}
}  // namespace navi_sim
