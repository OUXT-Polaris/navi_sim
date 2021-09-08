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

#ifndef NAVI_SIM__INTERPRETER__DATA_TYPE_HPP_
#define NAVI_SIM__INTERPRETER__DATA_TYPE_HPP_

#include <yaml-cpp/yaml.h>

#include <geometry_msgs/msg/pose_stamped.hpp>

namespace navi_sim
{
void parse(const YAML::Node & yaml, geometry_msgs::msg::PoseStamped & pose);
void parse(const YAML::Node & yaml, geometry_msgs::msg::Pose & pose);
void parse(const YAML::Node & yaml, geometry_msgs::msg::Point & point);
void parse(const YAML::Node & yaml, geometry_msgs::msg::Quaternion & quat);
}  // namespace navi_sim

#endif  // NAVI_SIM__INTERPRETER__DATA_TYPE_HPP_
