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

#include <navi_sim/interpreter/data_type.hpp>
#include <string>

namespace navi_sim
{
void parse(const YAML::Node & yaml, geometry_msgs::msg::PoseStamped & pose)
{
  pose.header.frame_id = yaml["frame"].as<std::string>();
  parse(yaml["pose"], pose.pose);
}

void parse(const YAML::Node & yaml, geometry_msgs::msg::Pose & pose)
{
  parse(yaml["position"], pose.position);
  parse(yaml["orientation"], pose.orientation);
}

void parse(const YAML::Node & yaml, geometry_msgs::msg::Point & point)
{
  point.x = yaml["x"].as<double>();
  point.y = yaml["y"].as<double>();
  point.z = yaml["z"].as<double>();
}

void parse(const YAML::Node & yaml, geometry_msgs::msg::Quaternion & quat)
{
  quat.x = yaml["x"].as<double>();
  quat.y = yaml["y"].as<double>();
  quat.z = yaml["z"].as<double>();
  quat.w = yaml["w"].as<double>();
}
}  // namespace navi_sim
