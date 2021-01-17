#include <navi_sim/lidar_sim_component.hpp>

namespace navi_sim
{
LidarSimComponent::LidarSimComponent(const rclcpp::NodeOptions & options)
: Node("navi_sim", options), buffer_(get_clock()), listener_(buffer_)
{
  pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("lidar_points", 1);
}

void LidarSimComponent::addObject(
  std::string name, geometry_msgs::msg::Pose pose,
  std::string model)
{
  navi_sim::Mesh mesh = models.load(model);
  raycaster_.addObject(name, pose, mesh);
}
}  // namespace navi_sim
