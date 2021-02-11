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

#include <navi_sim/camera_sim_component.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <rclcpp_components/register_node_macro.hpp>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <boost/geometry/geometries/box.hpp>

namespace navi_sim
{
CameraSimComponent::CameraSimComponent(const rclcpp::NodeOptions & options)
: Node("camera_sim", options), buffer_(get_clock()), listener_(buffer_)
{
  initialize();
}

CameraSimComponent::CameraSimComponent(std::string name, const rclcpp::NodeOptions & options)
: Node(name, options), buffer_(get_clock()), listener_(buffer_)
{
  initialize();
}

void CameraSimComponent::update()
{
  rclcpp::Time now = get_clock()->now();
  vision_msgs::msg::Detection2DArray detection_array;
  sensor_msgs::msg::CameraInfo camera_info;
  camera_info = camera_info_;
  camera_info.header.frame_id = camera_frame_;
  camera_info.header.stamp = now;
  detection_array.header.stamp = now;
  detection_array.header.frame_id = camera_frame_;
  try {
    geometry_msgs::msg::TransformStamped transform_stamped = buffer_.lookupTransform(
      map_frame_, camera_frame_, rclcpp::Time(0), tf2::durationFromSec(1.0));
    geometry_msgs::msg::Pose pose;
    pose.position.x = transform_stamped.transform.translation.x;
    pose.position.y = transform_stamped.transform.translation.y;
    pose.position.z = transform_stamped.transform.translation.z;
    pose.orientation = transform_stamped.transform.rotation;
  } catch (tf2::ExtrapolationException & ex) {
    RCLCPP_ERROR(get_logger(), ex.what());
  }
  typedef boost::geometry::model::d2::point_xy<double> point;
  typedef boost::geometry::model::polygon<point> polygon_type;
  typedef boost::geometry::model::box<point> box;
  box camera_bbox(point(0, 0), point(camera_info_.width, camera_info_.height));
  const auto names = raycaster_ptr_->getPrimitiveNames();
  for (const auto name : names) {
    const auto vertex = raycaster_ptr_->getVertex(name);
    polygon_type poly;
    typedef boost::geometry::ring_type<polygon_type>::type ring_type;
    ring_type & ring = boost::geometry::exterior_ring(poly);
    for (const auto & v : vertex) {
      cv::Point3d point_3d(v.x, v.y, v.z);
      cv::Point2d point_2d = cam_model_.project3dToPixel(point_3d);
      ring.push_back(point(point_2d.x, point_2d.y));
    }
    box bx;
    boost::geometry::envelope(poly, bx);
    box out;
    if (boost::geometry::intersection(camera_bbox, bx, out)) {
      vision_msgs::msg::Detection2D detection;
      detection.header.frame_id = camera_frame_;
      detection.header.stamp = now;
      detection.is_tracking = false;
      detection.bbox.center.x = (bx.max_corner().x() + bx.min_corner().x()) * 0.5;
      detection.bbox.center.y = (bx.max_corner().y() + bx.min_corner().y()) * 0.5;
      detection.bbox.size_x = bx.max_corner().x() - bx.min_corner().x();
      detection.bbox.size_y = bx.max_corner().y() - bx.min_corner().y();
      vision_msgs::msg::ObjectHypothesisWithPose result;
      result.id = raycaster_ptr_->getObjectType(name);
      result.score = 1.0;
      detection.results.emplace_back(result);
      detection_array.detections.emplace_back(detection);
    }
  }
  detection_pub_->publish(detection_array);
  camera_info_pub_->publish(camera_info);
}

void CameraSimComponent::initialize()
{
  double vertical_fov;
  declare_parameter("vertical_fov", 120.19512195121952);
  get_parameter("vertical_fov", vertical_fov);
  int horizontal_pixels;
  declare_parameter("horizontal_pixels", 720);
  get_parameter("horizontal_pixels", horizontal_pixels);
  int vertical_pixels;
  declare_parameter("vertical_pixels", 540);
  get_parameter("vertical_pixels", vertical_pixels);
  camera_info_ = sensor_msgs::msg::CameraInfo();
  camera_info_.height = vertical_pixels;
  camera_info_.width = vertical_pixels;
  camera_info_.distortion_model = "plumb_bob";
  camera_info_.d = {0, 0, 0, 0, 0};
  double f = static_cast<double>(vertical_pixels) * 0.5 /
    std::tan(vertical_fov * 0.5 / 180.0 * M_PI);
  camera_info_.k =
  {
    f, 0, static_cast<double>(horizontal_pixels) * 0.5,
    0, f, static_cast<double>(vertical_pixels) * 0.5,
    0, 0, 1
  };
  camera_info_.r =
  {
    1, 0, 0,
    0, 1, 0,
    0, 0, 1
  };
  camera_info_.p =
  {
    f, 0, static_cast<double>(horizontal_pixels) * 0.5, 0,
    0, f, static_cast<double>(vertical_pixels) * 0.5, 0,
    0, 0, 1, 0
  };
  cam_model_.fromCameraInfo(camera_info_);
  declare_parameter("objects_filename", "objects.json");
  std::string objects_filename;
  get_parameter("objects_filename", objects_filename);
  std::string objects_path = ament_index_cpp::get_package_share_directory("navi_sim") + "/config/" +
    objects_filename;
  namespace fs = boost::filesystem;
  const fs::path path(objects_path);
  boost::system::error_code error;
  const bool result = fs::exists(path, error);
  if (!result || error) {
    throw std::runtime_error(objects_path + " did not find");
  }
  std::ifstream fin;
  fin.open(objects_path, std::ios::in);
  std::string json_string = "";
  std::string line;
  while (std::getline(fin, line)) {
    json_string = json_string + line;
  }
  raycaster_ptr_->addPrimitives(nlohmann::json::parse(json_string));
  // vision_info_pub_ = create_publisher<vision_msgs::msg::VisionInfo>("vision_info", 1);
  detection_pub_ = create_publisher<vision_msgs::msg::Detection2DArray>("detection", 1);
  camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 1);
  using namespace std::chrono_literals;
  timer_ = create_wall_timer(100ms, std::bind(&CameraSimComponent::update, this));
}
}  // namespace navi_sim

RCLCPP_COMPONENTS_REGISTER_NODE(navi_sim::CameraSimComponent)
