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

#include <navi_sim/primitives/primitive.hpp>

#include <quaternion_operation/quaternion_operation.h>

#include <algorithm>
#include <string>
#include <vector>

namespace navi_sim
{
Primitive::Primitive(std::string type, geometry_msgs::msg::Pose pose)
: type(type), pose(pose) {}

Vertex Primitive::transform(Vertex v) const
{
  auto mat = quaternion_operation::getRotationMatrix(pose.orientation);
  Eigen::VectorXd point(3);
  point(0) = v.x;
  point(1) = v.y;
  point(2) = v.z;
  point = mat * point;
  point(0) = point(0) + pose.position.x;
  point(1) = point(1) + pose.position.y;
  point(2) = point(2) + pose.position.z;
  Vertex ret;
  ret.x = point(0);
  ret.y = point(1);
  ret.z = point(2);
  return ret;
}

std::vector<Vertex> Primitive::transform() const
{
  std::vector<Vertex> ret;
  for (auto & v : vertices_) {
    ret.emplace_back(transform(v));
  }
  return ret;
}

std::vector<Vertex> Primitive::getVertex() const
{
  return transform();
}

std::vector<Triangle> Primitive::getTriangles() const
{
  return triangles_;
}

unsigned int Primitive::addToScene(RTCDevice device, RTCScene scene)
{
  RTCGeometry mesh = rtcNewGeometry(device, RTC_GEOMETRY_TYPE_TRIANGLE);
  const auto transformed_vertices = transform();
  Vertex * vertices = static_cast<Vertex *>(rtcSetNewGeometryBuffer(
      mesh, RTC_BUFFER_TYPE_VERTEX, 0,
      RTC_FORMAT_FLOAT3, sizeof(Vertex), transformed_vertices.size()));
  for (size_t i = 0; i < transformed_vertices.size(); i++) {
    vertices[i] = transformed_vertices[i];
  }
  Triangle * triangles = static_cast<Triangle *>(rtcSetNewGeometryBuffer(
      mesh, RTC_BUFFER_TYPE_INDEX, 0,
      RTC_FORMAT_UINT3, sizeof(Triangle),
      triangles_.size()));
  for (size_t i = 0; i < triangles_.size(); i++) {
    triangles[i] = triangles_[i];
  }
  rtcCommitGeometry(mesh);
  unsigned int geometry_id = rtcAttachGeometry(scene, mesh);
  rtcReleaseGeometry(mesh);
  return geometry_id;
}

void to_json(nlohmann::json & j, const geometry_msgs::msg::Point & p)
{
  j = nlohmann::json{{"x", p.x}, {"y", p.y}, {"z", p.z}};
}

void from_json(const nlohmann::json & j, geometry_msgs::msg::Point & p)
{
  j.at("x").get_to(p.x);
  j.at("y").get_to(p.y);
  j.at("z").get_to(p.z);
}

void to_json(nlohmann::json & j, const geometry_msgs::msg::Quaternion & q)
{
  j = nlohmann::json{{"x", q.x}, {"y", q.y}, {"z", q.z}, {"w", q.w}};
}

void from_json(const nlohmann::json & j, geometry_msgs::msg::Quaternion & q)
{
  j.at("x").get_to(q.x);
  j.at("y").get_to(q.y);
  j.at("z").get_to(q.z);
  j.at("w").get_to(q.w);
}

void to_json(nlohmann::json & j, const geometry_msgs::msg::Pose & p)
{
  j = nlohmann::json{};
  auto j_position = nlohmann::json{};
  to_json(j_position, p.position);
  j["position"] = j_position;
  auto j_orientation = nlohmann::json{};
  to_json(j_orientation, p.orientation);
  j["orientation"] = j_orientation;
}

void from_json(const nlohmann::json & j, geometry_msgs::msg::Pose & p)
{
  from_json(j["position"], p.position);
  from_json(j["orientation"], p.orientation);
}
}  // namespace navi_sim
