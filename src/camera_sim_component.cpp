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

#include "navi_sim/camera_sim_component.hpp"

#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <boost/geometry.hpp>
#include <boost/geometry/geometries/box.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>
#include <color_names/color_names.hpp>
#include <memory>
#include <perception_msgs/msg/object_hypothesis_with_pose.hpp>
#include <rclcpp_components/register_node_macro.hpp>
#include <string>
#include <vector>

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
  perception_msgs::msg::Detection2DArray detection_array;
  sensor_msgs::msg::CameraInfo camera_info;
  camera_info = camera_info_;
  camera_info.header.frame_id = camera_optical_frame_;
  camera_info.header.stamp = now;
  detection_array.header.stamp = now;
  detection_array.header.frame_id = camera_optical_frame_;
  geometry_msgs::msg::TransformStamped transform_stamped;
  try {
    transform_stamped = buffer_.lookupTransform(
      camera_optical_frame_, map_frame_, rclcpp::Time(0), tf2::durationFromSec(1.0));
    geometry_msgs::msg::Pose pose;
    pose.position.x = transform_stamped.transform.translation.x;
    pose.position.y = transform_stamped.transform.translation.y;
    pose.position.z = transform_stamped.transform.translation.z;
    pose.orientation = transform_stamped.transform.rotation;
  } catch (tf2::ExtrapolationException & ex) {
    RCLCPP_ERROR(get_logger(), ex.what());
    return;
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
    bool is_back = false;
    for (const auto & v : vertex) {
      geometry_msgs::msg::PointStamped p;
      p.point.x = v.x;
      p.point.y = v.y;
      p.point.z = v.z;
      tf2::doTransform(p, p, transform_stamped);
      if (p.point.z <= 0) {
        is_back = true;
      }
      cv::Point3d point_3d(p.point.x, p.point.y, p.point.z);
      cv::Point2d point_2d = cam_model_.project3dToPixel(point_3d);
      ring.push_back(point(point_2d.x, point_2d.y));
    }
    if (is_back) {
      continue;
    }
    const box bx = boost::geometry::return_envelope<box>(poly);
    box out;
    if (boost::geometry::intersection(camera_bbox, bx, out)) {
      perception_msgs::msg::Detection2D detection;
      detection.header.frame_id = camera_optical_frame_;
      detection.header.stamp = now;
      // detection.is_tracking = false;
      detection.bbox.center.x = (out.max_corner().x() + out.min_corner().x()) * 0.5;
      detection.bbox.center.y = (out.max_corner().y() + out.min_corner().y()) * 0.5;
      detection.bbox.size_x = out.max_corner().x() - out.min_corner().x();
      detection.bbox.size_y = out.max_corner().y() - out.min_corner().y();
      perception_msgs::msg::ObjectHypothesisWithPose result;
      // result.id = raycaster_ptr_->getObjectType(name);
      // result.score = 1.0;
      detection_array.detections.emplace_back(detection);
    }
  }
  detection_pub_->publish(detection_array);
  camera_info_pub_->publish(camera_info);
  marker_pub_->publish(generateMarker(detection_array.detections));
}

void CameraSimComponent::initialize()
{
  declare_parameter<std::string>("embree_config", "");
  if (has_parameter("embree_config")) {
    std::string embree_config;
    get_parameter("embree_config", embree_config);
    raycaster_ptr_ = std::make_unique<Raycaster>(embree_config);
  } else {
    raycaster_ptr_ = std::make_unique<Raycaster>();
  }
  double vertical_fov;
  declare_parameter<double>("vertical_fov", 2.0978006228796881594);
  get_parameter("vertical_fov", vertical_fov);
  declare_parameter<int>("horizontal_pixels", 720);
  get_parameter("horizontal_pixels", horizontal_pixels_);
  declare_parameter<int>("vertical_pixels", 540);
  get_parameter("vertical_pixels", vertical_pixels_);
  declare_parameter<std::string>("frustum_color", "cyan");
  get_parameter("frustum_color", frustum_color_);
  declare_parameter<std::string>("detection_color", "limegreen");
  get_parameter("detection_color", detection_color_);
  declare_parameter<std::string>("camera_frame", "camera_link");
  get_parameter("camera_frame", camera_frame_);
  declare_parameter<std::string>("camera_optical_frame", "camera_optical_link");
  get_parameter("camera_optical_frame", camera_optical_frame_);
  declare_parameter<std::string>("map_frame", "map");
  get_parameter("map_frame", map_frame_);
  camera_info_ = sensor_msgs::msg::CameraInfo();
  camera_info_.height = vertical_pixels_;
  camera_info_.width = horizontal_pixels_;
  camera_info_.distortion_model = "plumb_bob";
  camera_info_.d = {0, 0, 0, 0, 0};
  double f = static_cast<double>(vertical_pixels_) * 0.5 / std::tan(vertical_fov * 0.5);
  camera_info_.k = {f, 0, static_cast<double>(horizontal_pixels_) * 0.5,
                    0, f, static_cast<double>(vertical_pixels_) * 0.5,
                    0, 0, 1};
  camera_info_.r = {1, 0, 0, 0, 1, 0, 0, 0, 1};
  camera_info_.p = {
    f,
    0,
    static_cast<double>(horizontal_pixels_) * 0.5,
    0,
    0,
    f,
    static_cast<double>(vertical_pixels_) * 0.5,
    0,
    0,
    0,
    1,
    0};
  cam_model_.fromCameraInfo(camera_info_);
  declare_parameter("objects_filename", "objects.json");
  std::string objects_filename;
  get_parameter("objects_filename", objects_filename);
  std::string objects_path =
    ament_index_cpp::get_package_share_directory("navi_sim") + "/config/" + objects_filename;
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
  detection_pub_ = create_publisher<perception_msgs::msg::Detection2DArray>("detection", 1);
  camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>("camera_info", 1);
  marker_pub_ = create_publisher<visualization_msgs::msg::MarkerArray>("marker", 1);
  using namespace std::chrono_literals;
  timer_ = create_wall_timer(100ms, std::bind(&CameraSimComponent::update, this));
}

const geometry_msgs::msg::Point CameraSimComponent::internallyDivide(
  const geometry_msgs::msg::Point & p0, const geometry_msgs::msg::Point & p1,
  double x_ratio_in_image, double y_ratio_in_image)
{
  geometry_msgs::msg::Point p;
  p.x = p0.x * (1 - x_ratio_in_image) + p1.x * x_ratio_in_image;
  p.y = p0.y * (1 - y_ratio_in_image) + p1.y * y_ratio_in_image;
  p.z = (p0.z + p1.z) * 0.5;
  return p;
}

const visualization_msgs::msg::MarkerArray CameraSimComponent::generateMarker(
  const std::vector<perception_msgs::msg::Detection2D> & detections)
{
  const auto now = get_clock()->now();
  visualization_msgs::msg::MarkerArray marker;
  visualization_msgs::msg::Marker frustum_marker;
  frustum_marker.header.stamp = now;
  frustum_marker.header.frame_id = camera_optical_frame_;
  frustum_marker.id = 0;
  frustum_marker.ns = "frustum";
  frustum_marker.action = visualization_msgs::msg::Marker::ADD;
  frustum_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  frustum_marker.scale.x = 0.01;
  frustum_marker.scale.y = 0.01;
  frustum_marker.scale.z = 0.01;
  frustum_marker.color = color_names::makeColorMsg(frustum_color_, 1.0);
  geometry_msgs::msg::Point point_origin;
  point_origin.x = 0;
  point_origin.y = 0;
  point_origin.z = 0;
  cv::Point3d point_lu = cam_model_.projectPixelTo3dRay(cv::Point2d(0, 0));
  geometry_msgs::msg::Point point_lu_msg;
  point_lu_msg.x = point_lu.x;
  point_lu_msg.y = point_lu.y;
  point_lu_msg.z = point_lu.z;
  frustum_marker.points.emplace_back(point_origin);
  frustum_marker.points.emplace_back(point_lu_msg);
  cv::Point3d point_ru = cam_model_.projectPixelTo3dRay(cv::Point2d(horizontal_pixels_, 0));
  geometry_msgs::msg::Point point_ru_msg;
  point_ru_msg.x = point_ru.x;
  point_ru_msg.y = point_ru.y;
  point_ru_msg.z = point_ru.z;
  frustum_marker.points.emplace_back(point_origin);
  frustum_marker.points.emplace_back(point_ru_msg);
  cv::Point3d point_lb = cam_model_.projectPixelTo3dRay(cv::Point2d(0, vertical_pixels_));
  geometry_msgs::msg::Point point_lb_msg;
  point_lb_msg.x = point_lb.x;
  point_lb_msg.y = point_lb.y;
  point_lb_msg.z = point_lb.z;
  frustum_marker.points.emplace_back(point_origin);
  frustum_marker.points.emplace_back(point_lb_msg);
  cv::Point3d point_rb =
    cam_model_.projectPixelTo3dRay(cv::Point2d(horizontal_pixels_, vertical_pixels_));
  geometry_msgs::msg::Point point_rb_msg;
  point_rb_msg.x = point_rb.x;
  point_rb_msg.y = point_rb.y;
  point_rb_msg.z = point_rb.z;
  frustum_marker.points.emplace_back(point_origin);
  frustum_marker.points.emplace_back(point_rb_msg);
  // markers for edge
  frustum_marker.points.emplace_back(point_lu_msg);
  frustum_marker.points.emplace_back(point_ru_msg);
  frustum_marker.points.emplace_back(point_ru_msg);
  frustum_marker.points.emplace_back(point_rb_msg);
  frustum_marker.points.emplace_back(point_rb_msg);
  frustum_marker.points.emplace_back(point_lb_msg);
  frustum_marker.points.emplace_back(point_lb_msg);
  frustum_marker.points.emplace_back(point_lu_msg);
  frustum_marker.frame_locked = true;
  marker.markers.emplace_back(frustum_marker);
  visualization_msgs::msg::Marker detection_marker;
  detection_marker.header.stamp = now;
  detection_marker.header.frame_id = camera_optical_frame_;
  detection_marker.id = 0;
  detection_marker.ns = "detection";
  detection_marker.action = visualization_msgs::msg::Marker::ADD;
  detection_marker.type = visualization_msgs::msg::Marker::LINE_LIST;
  detection_marker.scale.x = 0.01;
  detection_marker.scale.y = 0.01;
  detection_marker.scale.z = 0.01;
  detection_marker.color = color_names::makeColorMsg(detection_color_, 1.0);
  detection_marker.frame_locked = true;
  for (const auto & detection : detections) {
    cv::Point2d point_lu_obj(
      detection.bbox.center.x - detection.bbox.size_x * 0.5,
      detection.bbox.center.y - detection.bbox.size_y * 0.5);
    cv::Point3d lu_ray = cam_model_.projectPixelTo3dRay(point_lu_obj);
    geometry_msgs::msg::Point point_lu_obj_msg;
    point_lu_obj_msg.x = lu_ray.x;
    point_lu_obj_msg.y = lu_ray.y;
    point_lu_obj_msg.z = lu_ray.z;
    cv::Point2d point_ru_point(
      detection.bbox.center.x + detection.bbox.size_x * 0.5,
      detection.bbox.center.y - detection.bbox.size_y * 0.5);
    cv::Point3d ru_ray = cam_model_.projectPixelTo3dRay(point_ru_point);
    geometry_msgs::msg::Point point_ru_obj_msg;
    point_ru_obj_msg.x = ru_ray.x;
    point_ru_obj_msg.y = ru_ray.y;
    point_ru_obj_msg.z = ru_ray.z;
    cv::Point2d point_rb_point(
      detection.bbox.center.x + detection.bbox.size_x * 0.5,
      detection.bbox.center.y + detection.bbox.size_y * 0.5);
    cv::Point3d rb_ray = cam_model_.projectPixelTo3dRay(point_rb_point);
    geometry_msgs::msg::Point point_rb_obj_msg;
    point_rb_obj_msg.x = rb_ray.x;
    point_rb_obj_msg.y = rb_ray.y;
    point_rb_obj_msg.z = rb_ray.z;
    cv::Point2d point_lb_point(
      detection.bbox.center.x - detection.bbox.size_x * 0.5,
      detection.bbox.center.y + detection.bbox.size_y * 0.5);
    cv::Point3d lb_ray = cam_model_.projectPixelTo3dRay(point_lb_point);
    geometry_msgs::msg::Point point_lb_obj_msg;
    point_lb_obj_msg.x = lb_ray.x;
    point_lb_obj_msg.y = lb_ray.y;
    point_lb_obj_msg.z = lb_ray.z;
    detection_marker.points.emplace_back(point_lu_obj_msg);
    detection_marker.points.emplace_back(point_ru_obj_msg);
    detection_marker.points.emplace_back(point_ru_obj_msg);
    detection_marker.points.emplace_back(point_rb_obj_msg);
    detection_marker.points.emplace_back(point_rb_obj_msg);
    detection_marker.points.emplace_back(point_lb_obj_msg);
    detection_marker.points.emplace_back(point_lb_obj_msg);
    detection_marker.points.emplace_back(point_lu_obj_msg);
  }
  marker.markers.emplace_back(detection_marker);
  return marker;
}
}  // namespace navi_sim

RCLCPP_COMPONENTS_REGISTER_NODE(navi_sim::CameraSimComponent)
