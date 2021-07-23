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

#ifndef NAVI_SIM__INTERPRETER__BLACK_BOARD_HPP_
#define NAVI_SIM__INTERPRETER__BLACK_BOARD_HPP_

#include <boost/any.hpp>
#include <unordered_map>
#include <string>

namespace navi_sim
{
class BlackBoard
{
public:
  template<typename T>
  void set(const std::string & key, const T & value)
  {
    data_[key] = boost::any_cast<T>(value);
  }
  template<typename T>
  const T get(const std::string & key) const
  {
    return boost::any_cast<T>(getValue(key));
  }

private:
  boost::any getValue(const std::string & key);
  std::unordered_map<std::string, boost::any> data_;
};
}  // namespace navi_sim

#endif  // NAVI_SIM__INTERPRETER__BLACK_BOARD_HPP_
