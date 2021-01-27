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

#include <gtest/gtest.h>

#include <navi_sim/raycaster.hpp>

TEST(Model, loadModel)
{
  /*
  navi_sim::Models models;
  models.getPath("dock_block_2x2");
  navi_sim::Mesh dock_block = models.load("dock_block_2x2");
  navi_sim::Raycaster raycaster;
  raycaster.addObject("dock1", dock_block);
  raycaster.addObject("dock2", dock_block);
  EXPECT_EQ(dock_block.getNumVertices(), static_cast<size_t>(2664));
  EXPECT_EQ(dock_block.getIndices().size(), static_cast<size_t>(2448));
  geometry_msgs::msg::Pose origin;
  raycaster.raycast(origin, {});
  raycaster.raycast(origin, {});
  */
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
