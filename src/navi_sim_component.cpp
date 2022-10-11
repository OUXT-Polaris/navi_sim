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
#include <string>
#include <vector>

namespace navi_sim
{
NaviSimComponent::NaviSimComponent(const rclcpp::NodeOptions & options)
: Node("navi_sim", options),
  broadcaster_(this),
  buffer_(get_clock()),
  listener_(buffer_),
  engine_(seed_gen_())
{
  using namespace std::chrono_literals;
  update_position_timer_ =
    this->create_wall_timer(10ms, std::bind(&NaviSimComponent::updatePose, this));
  declare_parameter("with_covariance", false);
  get_parameter("with_covariance", with_covariance_);
  declare_parameter("publish_twist", true);
  get_parameter("publish_twist", publish_twist_);
  declare_parameter("publish_pose", true);
  get_parameter("publish_pose", publish_pose_);
  declare_parameter("position_covariance", 0.0);
  get_parameter("position_covariance", position_covariance_);
  declare_parameter("rpy_covariance", 0.0);
  get_parameter("rpy_covariance", rpy_covariance_);
  declare_parameter("linear_covariance", 0.0);
  get_parameter("linear_covariance", linear_covariance_);
  declare_parameter("angular_covariance", 0.0);
  get_parameter("angular_covariance", angular_covariance_);
  declare_parameter("hull.width", 1.0);
  get_parameter("hull.width", width_);
  declare_parameter("hull.mass", 10.0);
  get_parameter("hull.mass", mass_);
  declare_parameter("hull.additional_mass_x", 1.0);
  get_parameter("hull.additional_mass_x", additional_mass_x_);
  declare_parameter("hull.additional_mass_y", 1.0);
  get_parameter("hull.additional_mass_y", additional_mass_y_);
  declare_parameter("hull.inertia", 5.0);
  get_parameter("hull.inertia", inertia_);
  declare_parameter("hull.additional_inertia_z", 1.0);
  get_parameter("hull.additional_inertia_z", additional_inertia_z_);
  if (with_covariance_) {
    current_pose_with_covariance_pub_ =
      this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("current_pose", 1);
    current_twist_with_covariance_pub_ =
      this->create_publisher<geometry_msgs::msg::TwistWithCovarianceStamped>("current_twist", 1);
  } else {
    current_pose_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("current_pose", 1);
    current_twist_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("current_twist", 1);
  }
  joint_state_pub_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 1);
  initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
    "initialpose", 1,
    std::bind(&NaviSimComponent::initialPoseCallback, this, std::placeholders::_1));
  current_twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
    "current_twist", 1,
    std::bind(&NaviSimComponent::currentTwistCallback, this, std::placeholders::_1));
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
  std::vector<std::string> joints = {
    "left_chasis_engine_joint", "left_engine_propeller_joint", "right_chasis_engine_joint",
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
  const double dt = 0.01;

  // Simulate current thruster force

  // TCP/IPで回転数を受け取る src/description/wamv_description/urdf/wamv.urdf.xacro 
  // 一旦TCP/IP経由じゃなくて船の出す力[2]を直接subする形式にする
  // ip:   192.168.10.100
  // poer: 12345
  thruster_force_ = {0.0, 0.0}; // left, right;

  // Simulate current twist
  Eigen::Vector3d input_vec;
  Eigen::Vector3d current_twist_vec;
  Eigen::Matrix3d mass_mat_inv;
  Eigen::Matrix3d drag_mat;
  Eigen::Matrix3d fh = Eigen::Matrix3d::Zero(); // Todo
  input_vec <<
    0.5*(thruster_force_[1] + thruster_force_[0]),
    0,
    0.5*width_*(thruster_force_[1] - thruster_force_[0]);
  current_twist_vec <<
    current_twist_.linear.x,
    current_twist_.linear.y,
    current_twist_.angular.z;
  mass_mat_inv <<
    1/(mass_+additional_mass_x_), 0, 0,
    0, 1/(mass_+additional_mass_y_), 0,
    0, 0, 1/(inertia_+additional_mass_x_);
  drag_mat <<
    0, 0, -mass_*current_twist_vec(1),
    0, 0,  mass_*current_twist_vec(0),
    mass_*current_twist_vec(1), -mass_*current_twist_vec(0), 0;
  Eigen::Vector3d accel_vec = mass_mat_inv*( current_twist_vec- drag_mat*current_twist_vec + fh*current_twist_vec + input_vec );
  prev_twist_ = current_twist_;
  current_twist_.linear.x += accel_vec(0) * dt;
  current_twist_.linear.y += accel_vec(1) * dt;
  current_twist_.angular.z += accel_vec(2) * dt;

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

  if (with_covariance_) {
    if (publish_pose_) {
      current_pose_with_covariance_pub_->publish(applyNoise(current_pose_msg));
    }
    if (publish_twist_) {
      geometry_msgs::msg::TwistStamped twist_stamped;
      twist_stamped.header.stamp = get_clock()->now();
      twist_stamped.header.frame_id = "base_link";
      twist_stamped.twist = current_twist_;
      current_twist_with_covariance_pub_->publish(applyNoise(twist_stamped));
    }
    // current_twist_with_covariance_pub_->publish(current_twist_with_covariance_pub_());
  } else {
    if (publish_pose_) {
      current_pose_pub_->publish(current_pose_msg);
    }
    if (publish_twist_) {
      current_twist_pub_->publish(current_twist_);
    }
  }
  updateJointState();
}

geometry_msgs::msg::PoseWithCovarianceStamped NaviSimComponent::applyNoise(
  const geometry_msgs::msg::PoseStamped & pose)
{
  geometry_msgs::msg::PoseWithCovarianceStamped value;
  value.header = pose.header;
  value.pose.covariance.fill(0);
  value.pose.covariance[0] = position_covariance_;
  value.pose.covariance[3] = position_covariance_;
  value.pose.covariance[8] = position_covariance_;
  value.pose.covariance[15] = rpy_covariance_;
  value.pose.covariance[24] = rpy_covariance_;
  value.pose.covariance[35] = rpy_covariance_;
  std::normal_distribution<> dist_position(0.0, position_covariance_);
  value.pose.pose.position.x = pose.pose.position.x + dist_position(engine_);
  value.pose.pose.position.y = pose.pose.position.y + dist_position(engine_);
  value.pose.pose.position.z = pose.pose.position.z + dist_position(engine_);
  geometry_msgs::msg::Vector3 rpy_offset;
  std::normal_distribution<> dist_rpy(0.0, rpy_covariance_);
  rpy_offset.x = dist_rpy(engine_);
  rpy_offset.y = dist_rpy(engine_);
  rpy_offset.z = dist_rpy(engine_);
  value.pose.pose.orientation =
    pose.pose.orientation * quaternion_operation::convertEulerAngleToQuaternion(rpy_offset);
  return value;
}

geometry_msgs::msg::TwistWithCovarianceStamped NaviSimComponent::applyNoise(
  const geometry_msgs::msg::TwistStamped & twist)
{
  geometry_msgs::msg::TwistWithCovarianceStamped value;
  value.header = twist.header;
  value.twist.covariance.fill(0);
  value.twist.covariance[0] = linear_covariance_;
  value.twist.covariance[3] = linear_covariance_;
  value.twist.covariance[8] = linear_covariance_;
  value.twist.covariance[15] = angular_covariance_;
  value.twist.covariance[24] = angular_covariance_;
  value.twist.covariance[35] = angular_covariance_;
  std::normal_distribution<> dist_linear(0.0, linear_covariance_);
  value.twist.twist.linear.x = value.twist.twist.linear.x + dist_linear(engine_);
  value.twist.twist.linear.y = value.twist.twist.linear.y + dist_linear(engine_);
  value.twist.twist.linear.z = value.twist.twist.linear.z + dist_linear(engine_);
  std::normal_distribution<> dist_angular(0.0, angular_covariance_);
  value.twist.twist.angular.x = value.twist.twist.angular.x + dist_angular(engine_);
  value.twist.twist.angular.y = value.twist.twist.angular.y + dist_angular(engine_);
  value.twist.twist.angular.z = value.twist.twist.angular.z + dist_angular(engine_);
  return value;
}

void NaviSimComponent::currentTwistCallback(const geometry_msgs::msg::Twist::SharedPtr data)
{
  // RCLCPP_INFO(get_logger(), "cur twist sub");
  current_twist_ = *data;
}

void NaviSimComponent::targetTwistCallback(const geometry_msgs::msg::Twist::SharedPtr data)
{
  // 将来的にTCP/IPでの指令値受け取りが実装されたら使われなくなる関数
  // current_twist_ = *data;
  target_twist_ = *data;
}

void NaviSimComponent::initialPoseCallback(
  const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr data)
{
  current_pose_ = data->pose.pose;
  current_twist_ = geometry_msgs::msg::Twist();
}
}  // namespace navi_sim

RCLCPP_COMPONENTS_REGISTER_NODE(navi_sim::NaviSimComponent)
