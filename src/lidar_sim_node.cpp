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

// Headers in RCLCPP
#include <rclcpp/rclcpp.hpp>
#include <memory>

int main(int argc, char * argv[])
{
  /*
  navi_sim::Models models;
  models.getPath("dock_block_2x2");
  navi_sim::Mesh dock_block = models.load("dock_block_2x2");
  navi_sim::Raycaster raycaster;
  raycaster.addObject("dock1", dock_block);
  raycaster.addObject("dock2", dock_block);
  geometry_msgs::msg::Pose origin;
  geometry_msgs::msg::Quaternion direction;
  dock_block.getNumVertices();
  std::cout << __FILE__ << "," << __LINE__ << std::endl;
  raycaster.raycast(origin, {direction});
  std::cout << __FILE__ << "," << __LINE__ << std::endl;
  raycaster.raycast(origin, {direction});
  std::cout << __FILE__ << "," << __LINE__ << std::endl;
  raycaster.raycast(
    origin, 2 * M_PI / 360.0,
  {
    RAD2DEG(-15.0), RAD2DEG(-13.0), RAD2DEG(-11.0), RAD2DEG(-9.0),
    RAD2DEG(-7.0), RAD2DEG(-5.0), RAD2DEG(-3.0), RAD2DEG(-1.0),
    RAD2DEG(1.0), RAD2DEG(3.0), RAD2DEG(5.0), RAD2DEG(7.0),
    RAD2DEG(9.0), RAD2DEG(11.0), RAD2DEG(13.0), RAD2DEG(15.0)
  });
  */
  return 0;
}
