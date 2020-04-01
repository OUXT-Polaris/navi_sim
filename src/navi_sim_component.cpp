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

#include <navi_sim/navi_sim_component.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <set>
#include <vector>

namespace navi_sim
{
NaviSimComponent::NaviSimComponent(const rclcpp::NodeOptions & options)
: Node("navi_sim", options), broadcaster_(this), buffer_(get_clock()), listener_(buffer_)
{
  using namespace std::chrono_literals;
  obstacle_radius_ = 1.0;
  maximum_scan_range_ = 20.0;
  minimum_scan_range_ = 0.3;
  num_scans_ = 360;
  update_position_timer_ =
    this->create_wall_timer(10ms, std::bind(&NaviSimComponent::updatePose, this));
  update_scan_timer_ =
    this->create_wall_timer(10ms, std::bind(&NaviSimComponent::updateScan, this));
  current_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("current_pose", 1);
  current_twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("current_twist", 1);
  laser_pub_ = this->create_publisher<sensor_msgs::msg::LaserScan>("obstacle_scan", 1);
  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 1,
    std::bind(&NaviSimComponent::initialPoseCallback, this, std::placeholders::_1));
  target_twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "target_twist", 1,
    std::bind(&NaviSimComponent::targetTwistCallback, this, std::placeholders::_1));
  clicked_point_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
    "clicked_point", 1,
    std::bind(&NaviSimComponent::clickedPointCallback, this, std::placeholders::_1));
}

geometry_msgs::msg::PointStamped NaviSimComponent::TransformToMapFrame(
  geometry_msgs::msg::PointStamped point)
{
  if (point.header.frame_id == "map") {
    return point;
  }
  tf2::TimePoint time_point = tf2::TimePoint(
    std::chrono::seconds(point.header.stamp.sec) +
    std::chrono::nanoseconds(point.header.stamp.nanosec));
  geometry_msgs::msg::TransformStamped transform_stamped =
    buffer_.lookupTransform("map", point.header.frame_id, time_point, tf2::durationFromSec(1.0));
  tf2::doTransform(point, point, transform_stamped);
  return point;
}

geometry_msgs::msg::PointStamped NaviSimComponent::TransformToBaselinkFrame(
  geometry_msgs::msg::PointStamped point, bool from_message_timestamp)
{
  if (point.header.frame_id == "base_link") {
    return point;
  }
  tf2::TimePoint time_point;
  if (from_message_timestamp) {
    time_point = tf2::TimePoint(
      std::chrono::seconds(point.header.stamp.sec) +
      std::chrono::nanoseconds(point.header.stamp.nanosec));
  }
  geometry_msgs::msg::TransformStamped transform_stamped = buffer_.lookupTransform(
    "base_link", point.header.frame_id, time_point, tf2::durationFromSec(1.0));
  tf2::doTransform(point, point, transform_stamped);
  return point;
}

void NaviSimComponent::updatePose()
{
  mtx_.lock();
  // Update Current Pose
  geometry_msgs::msg::Vector3 angular_trans_vec;
  angular_trans_vec.z = current_twist_.angular.z * 0.01;
  geometry_msgs::msg::Quaternion angular_trans_quat =
    convertEulerAngleToQuaternion(angular_trans_vec);
  current_pose_.orientation =
    quaternion_operation::rotation(current_pose_.orientation, angular_trans_quat);
  Eigen::Vector3d trans_vec;
  trans_vec(0) = current_twist_.linear.x * 0.01;
  trans_vec(1) = current_twist_.linear.y * 0.01;
  Eigen::Matrix3d rotation_mat = quaternion_operation::getRotationMatrix(current_pose_.orientation);
  trans_vec = rotation_mat * trans_vec;
  current_pose_.position.x = trans_vec(0) + current_pose_.position.x;
  current_pose_.position.y = trans_vec(1) + current_pose_.position.y;

  // Publish tf
  geometry_msgs::msg::TransformStamped transform_stamped;
  transform_stamped.header.stamp = get_clock()->now();
  transform_stamped.header.frame_id = "map";
  transform_stamped.child_frame_id = "base_link";
  transform_stamped.transform.translation.x = current_pose_.position.x;
  transform_stamped.transform.translation.y = current_pose_.position.y;
  transform_stamped.transform.translation.z = 0.0;
  transform_stamped.transform.rotation = current_pose_.orientation;
  broadcaster_.sendTransform(transform_stamped);

  // Publish current Pose
  geometry_msgs::msg::PoseStamped current_pose_msg;
  current_pose_msg.pose = current_pose_;
  current_pose_msg.header.stamp = transform_stamped.header.stamp;
  current_pose_msg.header.frame_id = "map";
  current_pose_pub_->publish(current_pose_msg);
  current_twist_pub_->publish(current_twist_);
  mtx_.unlock();
}

void NaviSimComponent::targetTwistCallback(const geometry_msgs::msg::Twist::SharedPtr data)
{
  mtx_.lock();
  current_twist_ = *data;
  mtx_.unlock();
}

void NaviSimComponent::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr data)
{
  mtx_.lock();
  current_pose_ = data->pose.pose;
  current_twist_ = geometry_msgs::msg::Twist();
  mtx_.unlock();
}

void NaviSimComponent::clickedPointCallback(const geometry_msgs::msg::PointStamped::SharedPtr data)
{
  mtx_.lock();
  geometry_msgs::msg::PointStamped p = TransformToMapFrame(*data);
  obstacles_.push_back(p.point);
  mtx_.unlock();
}

boost::optional<double> NaviSimComponent::getDistanceToObstacle(
  geometry_msgs::msg::PointStamped obstacle, double theta)
{
  double x0 = obstacle.point.x;
  double y0 = obstacle.point.y;
  double a = std::pow(x0 * std::cos(theta) + y0 * std::sin(theta), 2) -
             (x0 * x0 + y0 * y0 - obstacle_radius_ * obstacle_radius_);
  if (a < 0.0) {
    return boost::none;
  }
  double l1 = x0 * std::cos(theta) + y0 * std::sin(theta) - std::sqrt(a);
  double l2 = x0 * std::cos(theta) + y0 * std::sin(theta) + std::sqrt(a);
  if (l1 > minimum_scan_range_ && maximum_scan_range_ > l1) {
    return l1;
  }
  if (l2 > minimum_scan_range_ && maximum_scan_range_ > l2) {
    return l2;
  }
  return boost::none;
}

void NaviSimComponent::updateScan()
{
  mtx_.lock();
  std::vector<geometry_msgs::msg::PointStamped> obstacles;
  auto stamp = get_clock()->now();

  for (auto itr = obstacles_.begin(); itr != obstacles_.end(); itr++) {
    geometry_msgs::msg::PointStamped obstacle;
    obstacle.point = *itr;
    obstacle.header.stamp = stamp;
    obstacle.header.frame_id = "map";
    obstacles.push_back(TransformToBaselinkFrame(obstacle, false));
  }
  double angle_increment = M_PI * 2 / static_cast<double>(num_scans_);
  sensor_msgs::msg::LaserScan scan;
  scan.header.frame_id = "base_link";
  scan.header.stamp = stamp;
  scan.angle_min = 0;
  scan.angle_max = 2 * M_PI;
  scan.angle_increment = angle_increment;
  scan.time_increment = 0;
  scan.scan_time = 0.1;
  scan.range_min = minimum_scan_range_;
  scan.range_max = maximum_scan_range_;
  for (int i = 0; i < num_scans_; i++) {
    std::set<double> dists;
    double theta = angle_increment * i;
    for (auto obstacle_itr = obstacles.begin(); obstacle_itr != obstacles.end(); obstacle_itr++) {
      auto dist = getDistanceToObstacle(*obstacle_itr, theta);
      if (dist) {
        dists.insert(dist.get());
      }
    }
    if (dists.size() == 0) {
      scan.ranges.push_back(0.0);
    } else {
      double dist = *dists.begin();
      scan.ranges.push_back(dist);
    }
  }
  laser_pub_->publish(scan);
  mtx_.unlock();
}
}  // namespace navi_sim

RCLCPP_COMPONENTS_REGISTER_NODE(navi_sim::NaviSimComponent)
