#include <navi_sim/navi_sim_component.h>

namespace navi_sim
{
    NaviSimComponent::NaviSimComponent(const rclcpp::NodeOptions & options)
    : Node("navi_sim", options),
        ros_clock_(RCL_ROS_TIME), 
        broadcaster_(this)
    {
        using namespace std::chrono_literals;
        update_position_timer_ = this->create_wall_timer(500ms, std::bind(&NaviSimComponent::updatePose, this));
        initial_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>
            ("initialpose", 1, std::bind(&NaviSimComponent::initialPoseCallback, this, std::placeholders::_1));
        target_twist_sub_ = this->create_subscription<geometry_msgs::msg::Twist>
            ("target_twist", 1, std::bind(&NaviSimComponent::targetTwistCallback, this, std::placeholders::_1));
    }

    void NaviSimComponent::updatePose()
    {
        mtx_.lock();
        // Update Current Pose
        using namespace quaternion_operation;
        convertEulerAngleToQuaternion(current_twist_.angular);
        // Publish tf
        geometry_msgs::msg::TransformStamped transform_stamped;
        transform_stamped.header.stamp = ros_clock_.now();
        transform_stamped.header.frame_id = "map";
        transform_stamped.child_frame_id = "base_link";
        transform_stamped.transform.translation.x = current_pose_.position.x;
        transform_stamped.transform.translation.y = current_pose_.position.y;
        transform_stamped.transform.translation.z = 0.0;
        transform_stamped.transform.rotation = current_pose_.orientation;
        broadcaster_.sendTransform(transform_stamped);
        mtx_.unlock();
    }

    void NaviSimComponent::targetTwistCallback(const geometry_msgs::msg::Twist::SharedPtr data)
    {
        mtx_.lock();
        current_twist_ = *data;
        mtx_.unlock();
    }

    void NaviSimComponent::initialPoseCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr data)
    {
        mtx_.lock();
        current_pose_ = data->pose.pose;
        current_twist_ = geometry_msgs::msg::Twist();
        mtx_.unlock();
    }
}