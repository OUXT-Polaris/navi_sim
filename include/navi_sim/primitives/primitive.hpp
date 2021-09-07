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

#include <embree3/rtcore.h>

#include <algorithm>
#include <geometry_msgs/msg/pose.hpp>
#include <nlohmann/json.hpp>
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
  unsigned int v0;
  unsigned int v1;
  unsigned int v2;
};

class Primitive
{
public:
  Primitive(std::string primitive_type, std::string object_type, geometry_msgs::msg::Pose pose);
  virtual ~Primitive() = default;
  const std::string primitive_type;
  const std::string object_type;
  const geometry_msgs::msg::Pose pose;
  unsigned int addToScene(RTCDevice device, RTCScene scene);
  std::vector<Vertex> getVertex() const;
  std::vector<Vertex> getVertex(const geometry_msgs::msg::Pose & sensor_pose) const;
  std::vector<Triangle> getTriangles() const;
  virtual nlohmann::json toJson() const { return nlohmann::json{}; }
  nlohmann::json toBaseJson() const;
  std::vector<geometry_msgs::msg::Point> get2DPolygon() const;

protected:
  std::vector<Vertex> transform(const geometry_msgs::msg::Pose & sensor_pose) const;
  std::vector<Vertex> transform() const;
  std::vector<Vertex> vertices_;
  std::vector<Triangle> triangles_;

private:
  Vertex transform(Vertex v) const;
  Vertex transform(Vertex v, const geometry_msgs::msg::Pose & sensor_pose) const;
};

void to_json(nlohmann::json & j, const Primitive & p);
void to_json(nlohmann::json & j, const geometry_msgs::msg::Point & p);
void from_json(const nlohmann::json & j, geometry_msgs::msg::Point & p);
void to_json(nlohmann::json & j, const geometry_msgs::msg::Quaternion & q);
void from_json(const nlohmann::json & j, geometry_msgs::msg::Quaternion & q);
void to_json(nlohmann::json & j, const geometry_msgs::msg::Pose & p);
void from_json(const nlohmann::json & j, geometry_msgs::msg::Pose & p);
}  // namespace navi_sim

#endif  // NAVI_SIM__PRIMITIVES__PRIMITIVE_HPP_
