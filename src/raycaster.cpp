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
#include <navi_sim/raycaster.hpp>

#include <unordered_map>
#include <string>
#include <algorithm>
#include <vector>
#include <iostream>

namespace navi_sim
{
Raycaster::Raycaster()
{
  objects_ = std::unordered_map<std::string, navi_sim::Mesh>();
}

Raycaster::~Raycaster()
{
}

void Raycaster::constractGeometry()
{
  geometry_vertices_ = std::vector<VertexData>(getNumVertices());
  geometry_indices_ = std::vector<PolygonIndexData>(getNumIndices());
  size_t current_vertices = 0;
  size_t current_polygon_index = 0;
  for (const auto object : objects_) {
    const auto vertex_points = object.second.getVertices();
    for (const auto vertex : vertex_points) {
      const auto p = vertex.getPosition();
      VertexData v = {static_cast<float>(p.x), static_cast<float>(p.y), static_cast<float>(p.z)};
      geometry_vertices_[current_vertices] = v;
      ++current_vertices;
    }
    const auto indices = object.second.getIndices();
    for (const auto index : indices) {
      PolygonIndexData i = {index[0], index[1], index[2]};
      geometry_indices_[current_polygon_index] = i;
      ++current_polygon_index;
    }
  }
}

void Raycaster::raycast()
{
  RTCDevice device_handle = rtcNewDevice(nullptr);
  RTCScene scene_handle = rtcNewScene(device_handle);
  RTCGeometry geometry_handle = rtcNewGeometry(device_handle, RTC_GEOMETRY_TYPE_TRIANGLE);
  rtcSetSharedGeometryBuffer(
    geometry_handle,
    RTC_BUFFER_TYPE_VERTEX,
    0,
    RTC_FORMAT_FLOAT3,
    geometry_vertices_.data(),
    0,
    sizeof(Vertex),
    geometry_vertices_.size());
  rtcSetSharedGeometryBuffer(
    geometry_handle,
    RTC_BUFFER_TYPE_INDEX,
    0,
    RTC_FORMAT_UINT3,
    geometry_indices_.data(),
    0,
    sizeof(PolygonIndexData),
    geometry_indices_.size());
  rtcCommitGeometry(geometry_handle);
  rtcAttachGeometry(scene_handle, geometry_handle);
  rtcReleaseGeometry(geometry_handle);
  rtcCommitScene(scene_handle);
  /**
   * @brief release handlers
   */
  rtcReleaseScene(scene_handle);
  rtcReleaseDevice(device_handle);
}

void Raycaster::addObject(std::string name, navi_sim::Mesh mesh)
{
  mesh.offsetIndex(getNumVertices());
  objects_.emplace(name, mesh);
  constractGeometry();
}

void Raycaster::addObject(std::string name, geometry_msgs::msg::Pose pose, navi_sim::Mesh mesh)
{
  mesh.transform(pose);
  addObject(name, mesh);
}

size_t Raycaster::getNumIndices() const
{
  size_t ret = 0;
  for (const auto object : objects_) {
    ret = ret + object.second.getNumIndices();
  }
  return ret;
}

size_t Raycaster::getNumVertices() const
{
  size_t ret = 0;
  for (const auto object : objects_) {
    ret = ret + object.second.getNumVertices();
  }
  return ret;
}
}  // namespace navi_sim
