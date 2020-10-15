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
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(quaternion_operation REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
)

add_library(navi_sim_component SHARED
  src/navi_sim_component.cpp
)
target_compile_definitions(navi_sim_component PRIVATE "NAVI_SIM_NAVI_SIM_COMPONENT_BUILDING_DLL")
target_link_libraries(navi_sim_component ${quaternion_operation_LIBRARIES})
ament_target_dependencies(navi_sim_component
  rclcpp rclcpp_components tf2_ros tf2_geometry_msgs quaternion_operation sensor_msgs visualization_msgs)

add_executable(navi_sim_node
  src/navi_sim_node.cpp
)
target_link_libraries(navi_sim_node navi_sim_component ${quaternion_operation_LIBRARIES})
ament_target_dependencies(navi_sim_node
  rclcpp rclcpp_components tf2_ros tf2_geometry_msgs quaternion_operation sensor_msgs visualization_msgs)

rclcpp_components_register_nodes(navi_sim_component
  "navi_sim::NaviSimComponent")

install(TARGETS
  navi_sim_node
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
install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
