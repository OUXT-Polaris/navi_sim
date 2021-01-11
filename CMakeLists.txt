cmake_minimum_required(VERSION 3.5)
project(navi_sim)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

find_package(Boost COMPONENTS filesystem REQUIRED)
link_directories(${Boost_LIBRARY_DIR})

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIR}
)

add_library(navi_sim_component SHARED
  src/navi_sim_component.cpp
  src/models.cpp
  src/mesh.cpp
  src/raycaster.cpp
  src/vertex.cpp
)
target_compile_definitions(navi_sim_component PRIVATE "NAVI_SIM_NAVI_SIM_COMPONENT_BUILDING_DLL")
target_link_libraries(navi_sim_component ${quaternion_operation_LIBRARIES} ${Boost_LIBRARIES})
ament_target_dependencies(navi_sim_component
  rclcpp
  rclcpp_components
  tf2_ros
  tf2_geometry_msgs
  quaternion_operation
  sensor_msgs
  visualization_msgs
  rviz_assimp_vendor
  embree_vendor)

add_executable(navi_sim_node
  src/navi_sim_node.cpp
)
target_link_libraries(navi_sim_node navi_sim_component ${quaternion_operation_LIBRARIES})
ament_target_dependencies(navi_sim_node
  rclcpp
  rclcpp_components
  tf2_ros
  tf2_geometry_msgs
  quaternion_operation
  sensor_msgs
  visualization_msgs
  rviz_assimp_vendor
  embree_vendor)

rclcpp_components_register_nodes(navi_sim_component
  "navi_sim::NaviSimComponent")

add_executable(lidar_sim_node
  src/lidar_sim_node.cpp
  src/models.cpp
  src/mesh.cpp
  src/raycaster.cpp
  src/vertex.cpp
)
ament_target_dependencies(lidar_sim_node
  rclcpp
  rclcpp_components
  tf2_ros
  tf2_geometry_msgs
  quaternion_operation
  sensor_msgs
  visualization_msgs
  rviz_assimp_vendor
  embree_vendor)
  target_link_libraries(lidar_sim_node ${quaternion_operation_LIBRARIES} ${Boost_LIBRARIES})

install(TARGETS
  navi_sim_node
  lidar_sim_node
  DESTINATION lib/navi_sim
)
install(TARGETS
  navi_sim_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
ament_export_libraries(navi_sim_component)

#############
## Install ##
#############
install(DIRECTORY launch config models
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_model test/src/test_model.cpp)
  target_link_libraries(test_model navi_sim_component)
endif()

ament_auto_package()
