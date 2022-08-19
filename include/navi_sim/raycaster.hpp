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

#ifndef NAVI_SIM__RAYCASTER_HPP_
#define NAVI_SIM__RAYCASTER_HPP_

#include <embree3/rtcore.h>
#include <pcl_conversions/pcl_conversions.h>

#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <memory>
#include <navi_sim/primitives/box.hpp>
#include <navi_sim/primitives/primitive.hpp>
#include <random>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

namespace navi_sim
{
class Raycaster
{
public:
  Raycaster();
  explicit Raycaster(std::string embree_config);
  ~Raycaster();
  void addPrimitives(nlohmann::json json);
  template <typename T, typename... Ts>
  void addPrimitive(std::string name, Ts &&... xs)
  {
    if (primitive_ptrs_.count(name) != 0) {
      throw std::runtime_error("primitive " + name + " already exist.");
    }
    auto primitive_ptr = std::make_unique<T>(std::forward<Ts>(xs)...);
    primitive_ptrs_.emplace(name, std::move(primitive_ptr));
  }
  const sensor_msgs::msg::PointCloud2 raycast(
    geometry_msgs::msg::Pose origin, double horizontal_resolution,
    std::vector<double> vertical_angles, double horizontal_angle_start = 0,
    double horizontal_angle_end = 2 * M_PI, double max_distance = 100, double min_distance = 0,
    double noise_distribution = 0.15, double ghost_ratio = 0.001);
  const sensor_msgs::msg::PointCloud2 raycast(
    geometry_msgs::msg::Pose origin, std::vector<geometry_msgs::msg::Quaternion> directions,
    double max_distance = 100, double min_distance = 0, double noise_distribution = 0.15,
    double ghost_ratio = 0.001);
  nlohmann::json dumpPrimitives() const;
  const std::vector<std::string> getPrimitiveNames();
  const std::string getPrimitiveType(const std::string & name);
  const std::string getObjectType(const std::string & name);
  const std::vector<Vertex> getVertex(const std::string & name);
  const std::vector<geometry_msgs::msg::Point> get2DPolygon(const std::string & name);
  const std::vector<std::string> queryByDistance(
    const geometry_msgs::msg::Point & origin, double distance) const;

private:
  std::unordered_map<std::string, std::unique_ptr<Primitive>> primitive_ptrs_;
  RTCDevice device_;
  RTCScene scene_;
  std::random_device seed_gen_;
  std::default_random_engine engine_;
};
}  // namespace navi_sim

#endif  // NAVI_SIM__RAYCASTER_HPP_
