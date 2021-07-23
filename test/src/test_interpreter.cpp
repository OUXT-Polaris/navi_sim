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

#include <navi_sim/interpreter/interpreter.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <string>

TEST(Interpreter, load)
{
  std::string path = ament_index_cpp::get_package_share_directory("navi_sim") +
    "/scenarios/go_straight.yaml";
  navi_sim::Interpreter interpreter(path);
  ASSERT_EQ(interpreter.getEventIndex("reach_first_goal"), static_cast<size_t>(0));
  ASSERT_EQ(interpreter.getEventIndex("reach_second_goal"), static_cast<size_t>(1));
}

int main(int argc, char ** argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
