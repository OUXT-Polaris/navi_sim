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

#ifndef NAVI_SIM__MODELS_HPP_
#define NAVI_SIM__MODELS_HPP_

#include <ament_index_cpp/get_package_share_directory.hpp>

#include <iostream>
#include <boost/filesystem.hpp>
#include <boost/foreach.hpp>

#include <vector>
#include <unordered_map>

namespace navi_sim
{
class Models
{
public:
  Models();
  std::vector<std::string> getModelNames() const;

private:
  std::unordered_map<std::string, std::string> dict_;
};
} // namespace navi_sim

#endif  // NAVI_SIM__MODELS_HPP_
