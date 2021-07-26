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

#include <yaml-cpp/yaml.h>

#include <iostream>

int main()
{
  const auto yaml = YAML::LoadFile("/tmp/context.yaml");
  std::cout << "========== Context ==========" << std::endl;
  std::cout << yaml << std::endl;
  std::cout << "========== Success or Failure ==========" << std::endl;
  if (yaml["actions"]["failure"]["state"].as<std::string>() == "finished") {
    std::cout << "failure!" << std::endl;
    std::exit(-1);
  }
  if (yaml["actions"]["success"]["state"].as<std::string>() == "finished") {
    std::cout << "success!" << std::endl;
    std::exit(0);
  }
  std::cout << "failure!" << std::endl;
  std::exit(-1);
}
