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

#include <navi_sim/primitives/primitive.hpp>

#include <quaternion_operation/quaternion_operation.h>

namespace navi_sim
{
Primitive::Primitive(std::string type, geometry_msgs::msg::Pose pose)
: type(type), pose(pose) {}

Vertex Primitive::transform(Vertex v)
{
  auto mat = quaternion_operation::getRotationMatrix(pose.orientation);
  Eigen::VectorXd point(3);
  point(0) = v.x;
  point(1) = v.y;
  point(2) = v.z;
  point = mat * point;
  point(0) = point(0) + pose.position.x;
  point(1) = point(1) + pose.position.y;
  point(2) = point(2) + pose.position.z;
  Vertex ret;
  ret.x = point(0);
  ret.y = point(1);
  ret.z = point(2);
}
}  // namespace navi_sim
