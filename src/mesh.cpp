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

#include <stdexcept>
#include <string>
#include <iostream>
#include <vector>

namespace navi_sim
{
Mesh::Mesh(std::string path)
{
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

void Mesh::initFromScene(const aiScene * scene_ptr)
{
  entries_.resize(scene_ptr->mNumMeshes);
  for (unsigned int i = 0; i < entries_.size(); i++) {
    const aiMesh * mesh_ptr = scene_ptr->mMeshes[i];
    initMesh(i, mesh_ptr);
  }
}

void Mesh::initMesh(unsigned int index, const aiMesh * mesh_ptr)
{
  entries_[index].material_index = mesh_ptr->mMaterialIndex;
  std::vector<Vertex> Vertices;
  std::vector<unsigned int> Indices;
  const aiVector3D Zero3D(0.0f, 0.0f, 0.0f);
  for (unsigned int i = 0; i < mesh_ptr->mNumVertices; i++) {
    const aiVector3D * position_ptr = &(mesh_ptr->mVertices[i]);
    const aiVector3D * normal_ptr = &(mesh_ptr->mNormals[i]);
    const aiVector3D * texture_coordinate_ptr =
      mesh_ptr->HasTextureCoords(0) ? &(mesh_ptr->mTextureCoords[0][i]) : &Zero3D;
    Vertex v(std::array<float, 3>{position_ptr->x, position_ptr->y, position_ptr->z},
      std::array<float, 2>{texture_coordinate_ptr->x, texture_coordinate_ptr->y},
      std::array<float, 3>{normal_ptr->x, normal_ptr->y, normal_ptr->z});
    Vertices.push_back(v);
  }
}

Mesh::MeshEntry::MeshEntry()
{
  num_indices = 0;
  material_index = INVALID_MATERIAL;
}
}  // namespace navi_sim
