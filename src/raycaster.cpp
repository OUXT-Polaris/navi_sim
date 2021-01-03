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

namespace navi_sim
{
Raycaster::Raycaster()
{
  device_handle_ = rtcNewDevice(nullptr);
  objects_ = std::unordered_map<std::string, navi_sim::Mesh>();
}

Raycaster::~Raycaster()
{
  rtcReleaseDevice(device_handle_);
}

void Raycaster::addObject(std::string name, geometry_msgs::msg::Pose pose, navi_sim::Mesh mesh)
{
}
}  // namespace navi_sim