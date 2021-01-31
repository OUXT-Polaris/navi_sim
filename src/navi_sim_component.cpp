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
#include <string>

namespace navi_sim
{
NaviSimComponent::NaviSimComponent(const rclcpp::NodeOptions & options)
: Node("navi_sim", options), broadcaster_(this), buffer_(get_clock()), listener_(buffer_)
{
  using namespace std::chrono_literals;
  update_position_timer_ =
    this->create_wall_timer(10ms, std::bind(&NaviSimComponent::updatePose, this));
  current_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("current_pose", 1);
  current_twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("current_twist", 1);
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 1,
    std::bind(&NaviSimComponent::initialPoseCallback, this, std::placeholders::_1));
  target_twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "target_twist", 1,
    std::bind(&NaviSimComponent::targetTwistCallback, this, std::placeholders::_1));
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

void NaviSimComponent::updateJointState()
{
  sensor_msgs::msg::JointState joint_state;
  joint_state.header.stamp = get_clock()->now();
  std::vector<std::string> joints =
  {"left_chasis_engine_joint", "left_engine_propeller_joint", "right_chasis_engine_joint",
    "right_engine_propeller_joint"};
  for (auto itr = joints.begin(); itr != joints.end(); itr++) {
    joint_state.name.push_back(*itr);
    joint_state.position.push_back(0.0);
    joint_state.velocity.push_back(0.0);
    joint_state.effort.push_back(0.0);
  }
  joint_state_pub_->publish(joint_state);
}

void NaviSimComponent::updatePose()
{
  // Update Current Pose
  geometry_msgs::msg::Vector3 angular_trans_vec;
  angular_trans_vec.z = current_twist_.angular.z * 0.01;
  geometry_msgs::msg::Quaternion angular_trans_quat =
    quaternion_operation::convertEulerAngleToQuaternion(angular_trans_vec);
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

  updateJointState();
}

void NaviSimComponent::targetTwistCallback(const geometry_msgs::msg::Twist::SharedPtr data)
{
  current_twist_ = *data;
}

void NaviSimComponent::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr data)
{
  current_pose_ = data->pose.pose;
  current_twist_ = geometry_msgs::msg::Twist();
}
}  // namespace navi_sim

RCLCPP_COMPONENTS_REGISTER_NODE(navi_sim::NaviSimComponent)
