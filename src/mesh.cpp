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

#include <navi_sim/mesh.hpp>

#include <quaternion_operation/quaternion_operation.h>

#include <Eigen/Core>

#include <stdexcept>
#include <string>
#include <iostream>
#include <vector>
#include <algorithm>

namespace navi_sim
{
Mesh::Mesh(std::string path)
{
  vertices_ = {};
  // entries_ = {};
  Assimp::Importer importer;
  const aiScene * scene_ptr = importer.ReadFile(
    path,
    aiProcess_Triangulate | aiProcess_GenSmoothNormals | aiProcess_FlipUVs |
    aiProcess_JoinIdenticalVertices);
  if (!scene_ptr) {
    throw std::runtime_error(importer.GetErrorString());
  }
  initFromScene(scene_ptr);
}

void Mesh::transform(const geometry_msgs::msg::Pose & pose)
{
  const auto mat = quaternion_operation::getRotationMatrix(pose.orientation);
  for (size_t i = 0; i < vertices_.size(); i++) {
    Eigen::VectorXd v(3);
    v(0) = vertices_[i].getPosition().x;
    v(1) = vertices_[i].getPosition().y;
    v(2) = vertices_[i].getPosition().z;
    v = mat * v;
    v(0) = v(0) + pose.position.x;
    v(1) = v(1) + pose.position.y;
    v(2) = v(2) + pose.position.z;
    geometry_msgs::msg::Point transformed;
    transformed.x = v(0);
    transformed.y = v(1);
    transformed.z = v(2);
    vertices_[i].setPosition(transformed);
  }
}

size_t Mesh::getNumVertices() const
{
  return vertices_.size();
}

const std::vector<navi_sim::Vertex> Mesh::getVertices() const
{
  return vertices_;
}

const std::vector<geometry_msgs::msg::Point> Mesh::getVerticesAsPoints() const
{
  std::vector<geometry_msgs::msg::Point> points;
  for (const auto vertex : vertices_) {
    points.emplace_back(vertex.getPosition());
  }
  return points;
}

size_t Mesh::getNumIndices() const
{
  return indices_.size();
}

const std::vector<std::array<unsigned int, 3>> Mesh::getIndices() const
{
  return indices_;
}

void Mesh::offsetIndex(unsigned int offset)
{
  for (auto i : indices_) {
    i[0] = i[0] + offset;
    i[1] = i[1] + offset;
    i[2] = i[2] + offset;
  }
}

void Mesh::initFromScene(const aiScene * scene_ptr)
{
  for (unsigned int i = 0; i < scene_ptr->mNumMeshes; i++) {
    const aiMesh * mesh_ptr = scene_ptr->mMeshes[i];
    initMesh(mesh_ptr);
  }
}

void Mesh::initMesh(const aiMesh * mesh_ptr)
{
  const aiVector3D Zero3D(0.0f, 0.0f, 0.0f);
  for (unsigned int i = 0; i < mesh_ptr->mNumVertices; i++) {
    const aiVector3D * position_ptr = &(mesh_ptr->mVertices[i]);
    const aiVector3D * texture_coordinate_ptr =
      mesh_ptr->HasTextureCoords(0) ? &(mesh_ptr->mTextureCoords[0][i]) : &Zero3D;
    geometry_msgs::msg::Point p;
    p.x = position_ptr->x;
    p.y = position_ptr->y;
    p.z = position_ptr->z;
    navi_sim::Vertex v(p, texture_coordinate_ptr->x, texture_coordinate_ptr->y);
    vertices_.push_back(v);
  }
  for (unsigned int i = 0; i < mesh_ptr->mNumFaces; i++) {
    const aiFace & face = mesh_ptr->mFaces[i];
    if (face.mNumIndices != 3) {
      throw("index of face should be three.");
    }
    std::array<unsigned int, 3> index_list = {face.mIndices[0], face.mIndices[1], face.mIndices[2]};
    indices_.emplace_back(index_list);
  }
}
}  // namespace navi_sim
