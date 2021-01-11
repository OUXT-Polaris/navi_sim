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

#ifndef NAVI_SIM__MESH_HPP_
#define NAVI_SIM__MESH_HPP_

#include <navi_sim/vertex.hpp>

#include <geometry_msgs/msg/pose.hpp>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <string>
#include <vector>
#include <array>

#define INVALID_MATERIAL 0xFFFFFFFF

namespace navi_sim
{
class Mesh
{
public:
  explicit Mesh(std::string path);
  void transform(const geometry_msgs::msg::Pose & pose);
  void offsetIndex(unsigned int offset);
  size_t getNumVertices() const;
  const std::vector<navi_sim::Vertex> getVertices() const;
  const std::vector<std::array<unsigned int, 3>> getIndices() const;

private:
  void initFromScene(const aiScene * scene_ptr);
  void initMesh(const aiMesh * mesh_ptr);
  std::vector<navi_sim::Vertex> vertices_;
  std::vector<std::array<unsigned int, 3>> indices_;
};
}  // namespace navi_sim

#endif  // NAVI_SIM__MESH_HPP_
