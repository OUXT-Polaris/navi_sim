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
#include <navi_sim/raycaster.hpp>

#include <quaternion_operation/quaternion_operation.h>

#include <unordered_map>
#include <string>
#include <algorithm>
#include <vector>
#include <iostream>

namespace navi_sim
{
Raycaster::Raycaster()
{
  // objects_ = std::unordered_map<std::string, navi_sim::Mesh>();
}

Raycaster::~Raycaster()
{
}

const sensor_msgs::msg::PointCloud2 Raycaster::raycast(
  geometry_msgs::msg::Pose origin,
  double horizontal_resolution,
  std::vector<double> vertical_angles,
  double max_distance, double min_distance
)
{
  std::vector<geometry_msgs::msg::Quaternion> directions;
  double horizontal_angle = 0;
  while (horizontal_angle <= (2 * M_PI)) {
    horizontal_angle = horizontal_angle + horizontal_resolution;
    for (const auto vertical_angle : vertical_angles) {
      geometry_msgs::msg::Vector3 rpy;
      rpy.x = 0;
      rpy.y = vertical_angle;
      rpy.z = horizontal_angle;
      auto quat = quaternion_operation::convertEulerAngleToQuaternion(rpy);
      directions.emplace_back(quat);
    }
  }
  return raycast(origin, directions, max_distance, min_distance);
}

const sensor_msgs::msg::PointCloud2 Raycaster::raycast(
  geometry_msgs::msg::Pose origin,
  std::vector<geometry_msgs::msg::Quaternion> directions,
  double max_distance, double min_distance)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>());
  RTCDevice device_handle = rtcNewDevice(nullptr);
  /*
  RTCScene scene_handle = rtcNewScene(device_handle);
  RTCGeometry geometry_handle = rtcNewGeometry(device_handle, RTC_GEOMETRY_TYPE_TRIANGLE);
  rtcSetSharedGeometryBuffer(
    geometry_handle,
    RTC_BUFFER_TYPE_VERTEX,
    0,
    RTC_FORMAT_FLOAT3,
    geometry_vertices_.data(),
    0,
    sizeof(Vertex),
    geometry_vertices_.size());
  rtcSetSharedGeometryBuffer(
    geometry_handle,
    RTC_BUFFER_TYPE_INDEX,
    0,
    RTC_FORMAT_UINT3,
    geometry_indices_.data(),
    0,
    sizeof(PolygonIndexData),
    geometry_indices_.size());
  rtcCommitGeometry(geometry_handle);
  rtcAttachGeometry(scene_handle, geometry_handle);
  rtcReleaseGeometry(geometry_handle);
  rtcCommitScene(scene_handle);
  RTCIntersectContext context;
  rtcInitIntersectContext(&context);
  RTCRayHit rayhit;
  for (auto direction : directions) {
    rayhit.ray.org_x = origin.position.x;
    rayhit.ray.org_y = origin.position.y;
    rayhit.ray.org_z = origin.position.z;
    rayhit.ray.tfar = max_distance;
    rayhit.ray.tnear = min_distance;
    rayhit.ray.flags = false;
    direction = origin.orientation * direction;
    const auto rotation_mat = quaternion_operation::getRotationMatrix(direction);
    const auto rotated_direction = rotation_mat * Eigen::Vector3d(1.0f, 0.0f, 0.0f);
    rayhit.ray.dir_x = rotated_direction[0];
    rayhit.ray.dir_y = rotated_direction[1];
    rayhit.ray.dir_z = rotated_direction[2];
    rayhit.hit.geomID = RTC_INVALID_GEOMETRY_ID;
    rtcIntersect1(scene_handle, &context, &rayhit);
    if (rayhit.hit.geomID != RTC_INVALID_GEOMETRY_ID) {
      double distance = rayhit.ray.tfar;
      const auto vector = rotated_direction * distance;
      pcl::PointXYZI p;
      p.x = origin.position.x + vector[0];
      p.y = origin.position.y + vector[1];
      p.z = origin.position.z + vector[2];
      cloud->emplace_back(p);
    }
  }
  rtcReleaseScene(scene_handle);
  rtcReleaseDevice(device_handle);
  sensor_msgs::msg::PointCloud2 pointcloud_msg;
  pcl::toROSMsg(*cloud, pointcloud_msg);
  return pointcloud_msg;
  */
}
}  // namespace navi_sim
