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

#ifndef NAVI_SIM__RAYCASTER_HPP_
#define NAVI_SIM__RAYCASTER_HPP_

#include <navi_sim/mesh.hpp>

#include <embree3/rtcore.h>

#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <unordered_map>
#include <vector>
#include <string>

namespace navi_sim
{

class Raycaster
{
public:
  Raycaster();
  ~Raycaster();
  void addObject(std::string name, geometry_msgs::msg::Pose pose, navi_sim::Mesh mesh);
  void addObject(std::string name, navi_sim::Mesh mesh);
  const sensor_msgs::msg::PointCloud2 raycast(
    geometry_msgs::msg::Point origin,
    std::vector<geometry_msgs::msg::Quaternion> directions,
    double max_distance = 100, double min_distance = 0);

private:
  std::unordered_map<std::string, navi_sim::Mesh> objects_;
  size_t getNumVertices() const;
  size_t getNumIndices() const;

  struct VertexData
  {
    float x, y, z;
  };

  struct PolygonIndexData
  {
    unsigned int v0, v1, v2;
  };

  std::vector<VertexData> geometry_vertices_;
  std::vector<PolygonIndexData> geometry_indices_;
  void constractGeometry();
};
}  // namespace navi_sim

#endif  // NAVI_SIM__RAYCASTER_HPP_
