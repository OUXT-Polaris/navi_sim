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

namespace navi_sim
{
Mesh::Mesh(std::string path)
{
  std::cout << __FILE__ << "," << __LINE__ << std::endl;
  std::cout << path << std::endl;
  Assimp::Importer Importer;
  const aiScene * pScene = Importer.ReadFile(
    path,
    aiProcess_Triangulate | aiProcess_GenSmoothNormals | aiProcess_FlipUVs |
    aiProcess_JoinIdenticalVertices);
  if (!pScene) {
    throw std::runtime_error(Importer.GetErrorString());
  }
}

bool Mesh::initFromScene(const aiScene * scene_ptr, const std::string & path)
{
  entries_.resize(scene_ptr->mNumMeshes);
  for (unsigned int i = 0; i < entries_.size(); i++) {
    const aiMesh * ai_mesh_ptr = scene_ptr->mMeshes[i];
    // InitMesh(i, paiMesh);
  }
}

Mesh::MeshEntry::MeshEntry()
{
  num_indices = 0;
  material_index = INVALID_MATERIAL;
}
}  // namespace navi_sim
