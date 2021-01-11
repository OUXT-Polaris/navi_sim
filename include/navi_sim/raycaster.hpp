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

#include <geometry_msgs/msg/pose.hpp>

#include <unordered_map>

namespace navi_sim
{
class Raycaster
{
public:
  Raycaster();
  ~Raycaster();
  void addObject(std::string name, geometry_msgs::msg::Pose pose, navi_sim::Mesh mesh);

private:
  RTCDevice device_handle_;
  RTCScene scene_handle_;
  RTCGeometry geometry_handle_;
  std::unordered_map<std::string, navi_sim::Mesh> objects_;
  size_t getNumVertices() const;
};
}  // namespace navi_sim

#endif  // NAVI_SIM__RAYCASTER_HPP_
