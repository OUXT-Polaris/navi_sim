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

#include <embree3/rtcore.h>

#include <nlohmann/json.hpp>

#include <string>
#include <vector>
#include <algorithm>

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
  unsigned int v0;
  unsigned int v1;
  unsigned int v2;
};

class Primitive
{
public:
  Primitive(std::string type, geometry_msgs::msg::Pose pose);
  const std::string type;
  const geometry_msgs::msg::Pose pose;
  unsigned int addToScene(RTCDevice device, RTCScene scene);
  std::vector<Vertex> getVertex() const;
  std::vector<Triangle> getTriangles() const;

protected:
  std::vector<Vertex> transform() const;
  std::vector<Vertex> vertices_;
  std::vector<Triangle> triangles_;

private:
  Vertex transform(Vertex v) const;
};

void to_json(nlohmann::json & j, const geometry_msgs::msg::Point & p);
void from_json(const nlohmann::json & j, geometry_msgs::msg::Point & p);
void to_json(nlohmann::json & j, const geometry_msgs::msg::Quaternion & q);
void from_json(const nlohmann::json & j, geometry_msgs::msg::Quaternion & q);
void to_json(nlohmann::json & j, const geometry_msgs::msg::Pose & p);
void from_json(const nlohmann::json & j, geometry_msgs::msg::Pose & p);
}  // namespace navi_sim

#endif  // NAVI_SIM__PRIMITIVES__PRIMITIVE_HPP_
