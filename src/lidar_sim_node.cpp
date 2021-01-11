// Copyright (c) 2020 OUXT Polaris
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

// Headers in this package
#include <navi_sim/raycaster.hpp>
#include <navi_sim/models.hpp>

// Headers in RCLCPP
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char * argv[])
{
  navi_sim::Models models;
  models.getPath("dock_block_2x2");
  navi_sim::Mesh dock_block = models.load("dock_block_2x2");
  navi_sim::Raycaster raycaster;
  raycaster.addObject("dock1", dock_block);
  raycaster.addObject("dock2", dock_block);
  geometry_msgs::msg::Point origin;
  dock_block.getNumVertices();
  std::cout << __FILE__ << "," << __LINE__ << std::endl;
  raycaster.raycast(origin, {});
  std::cout << __FILE__ << "," << __LINE__ << std::endl;
  raycaster.raycast(origin, {});
  std::cout << __FILE__ << "," << __LINE__ << std::endl;
  return 0;
}
