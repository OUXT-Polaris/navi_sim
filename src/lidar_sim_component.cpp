#include <navi_sim/lidar_sim_component.hpp>

namespace navi_sim
{
LidarSimComponent::LidarSimComponent(const rclcpp::NodeOptions & options)
: Node("navi_sim", options), buffer_(get_clock()), listener_(buffer_)
{
  using namespace std::chrono_literals;

  declare_parameter("lidar_frame", "base_link");
  get_parameter("lidar_frame", lidar_frame_);
  declare_parameter("map_frame", "map");
  get_parameter("map_frame", map_frame_);
  pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("lidar_points", 1);
  update_scan_timer_ =
    this->create_wall_timer(100ms, std::bind(&LidarSimComponent::updateScan, this));
  // marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("objects/marker", 1);
}

void LidarSimComponent::addObject(
  std::string name, geometry_msgs::msg::Pose pose,
  std::string model)
{
  navi_sim::Mesh mesh = models.load(model);
  raycaster_.addObject(name, pose, mesh);
}

void LidarSimComponent::updateScan()
{
  auto now = get_clock()->now();
  try {
    geometry_msgs::msg::TransformStamped transform_stamped = buffer_.lookupTransform(
      map_frame_, lidar_frame_, now, tf2::durationFromSec(1.0));
    geometry_msgs::msg::Pose pose;
    pose.position.x = transform_stamped.transform.translation.x;
    pose.position.y = transform_stamped.transform.translation.y;
    pose.position.z = transform_stamped.transform.translation.z;
    pose.orientation = transform_stamped.transform.rotation;
    const auto pointcloud_msg = raycaster_.raycast(
      pose, 2 * M_PI / 360.0,
      {
        RAD2DEG(-15.0), RAD2DEG(-13.0), RAD2DEG(-11.0), RAD2DEG(-9.0),
        RAD2DEG(-7.0), RAD2DEG(-5.0), RAD2DEG(-3.0), RAD2DEG(-1.0),
        RAD2DEG(1.0), RAD2DEG(3.0), RAD2DEG(5.0), RAD2DEG(7.0),
        RAD2DEG(9.0), RAD2DEG(11.0), RAD2DEG(13.0), RAD2DEG(15.0)
      });
    pointcloud_pub_->publish(pointcloud_msg);
  } catch (tf2::ExtrapolationException & ex) {
    RCLCPP_ERROR(get_logger(), ex.what());
  }
}
}  // namespace navi_sim
