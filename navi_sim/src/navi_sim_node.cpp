// Headers in this package
#include <navi_sim/navi_sim_component.hpp>
// Headers in RCLCPP
#include <rclcpp/rclcpp.hpp>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<navi_sim::NaviSimComponent>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}
