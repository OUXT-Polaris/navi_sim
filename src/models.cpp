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

#include <navi_sim/models.hpp>

namespace navi_sim
{
Models::Models()
{
  std::string pkg_share_path =
    ament_index_cpp::get_package_share_directory(
    "navi_sim");
  namespace fs = boost::filesystem;
  const fs::path path(pkg_share_path);
  BOOST_FOREACH(
    const fs::path & p, std::make_pair(
      fs::recursive_directory_iterator(path),
      fs::recursive_directory_iterator())) {
    if (!fs::is_directory(p)) {
      if (p.extension() == ".dae") {
        std::string model_name = p.parent_path().parent_path().filename().c_str();
        std::string model_path = p.c_str();
        dict_.emplace(model_name, model_path);
      }
    }
  }
}
} // namespace navi_sim
