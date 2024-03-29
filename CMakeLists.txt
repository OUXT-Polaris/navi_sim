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

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O0")

add_definitions("-DBOOST_ALLOW_DEPRECATED_HEADERS")

if($ENV{ROS_DISTRO} STREQUAL "galactic")
  add_definitions(-DGALACTIC)
endif()

if($ENV{ROS_DISTRO} STREQUAL "humble")
  add_definitions(-DHUMBLE)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(PCL REQUIRED)
find_package(perception_msgs REQUIRED)
ament_auto_find_build_dependencies()

if(${tf2_geometry_msgs_VERSION} VERSION_LESS 0.18.0)
  add_compile_definitions(USE_TF2_GEOMETRY_MSGS_DEPRECATED_HEADER)
endif()

find_package(Boost COMPONENTS filesystem REQUIRED)
link_directories(${Boost_LIBRARY_DIR} ${PCL_LIBRARY_DIRS})

include_directories(
  include
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIR}
  ${PCL_INCLUDE_DIRS}
)

ament_auto_add_library(navi_sim_component SHARED
  src/navi_sim_component.cpp
)
target_compile_definitions(navi_sim_component PRIVATE "NAVI_SIM_NAVI_SIM_COMPONENT_BUILDING_DLL")
target_link_libraries(navi_sim_component ${Boost_LIBRARIES})

ament_auto_add_executable(navi_sim_node
  src/navi_sim_node.cpp
)
target_link_libraries(navi_sim_node navi_sim_component)
rclcpp_components_register_nodes(navi_sim_component
  "navi_sim::NaviSimComponent")

ament_auto_add_library(lidar_sim_component SHARED
  src/lidar_sim_component.cpp
  src/lidar_sim_node.cpp
  src/primitives/box.cpp
  src/primitives/primitive.cpp
  src/raycaster.cpp
)
target_compile_definitions(lidar_sim_component PRIVATE "NAVI_SIM_LIDAR_SIM_COMPONENT_BUILDING_DLL")
target_link_libraries(lidar_sim_component embree3)

ament_auto_add_executable(lidar_sim_node
  src/lidar_sim_node.cpp
)
target_link_libraries(lidar_sim_node lidar_sim_component)
rclcpp_components_register_nodes(lidar_sim_component
  "navi_sim::LidarSimComponent")

ament_auto_add_library(camera_sim_component SHARED
  src/camera_sim_component.cpp
  src/primitives/box.cpp
  src/primitives/primitive.cpp
  src/raycaster.cpp
)
target_compile_definitions(camera_sim_component PRIVATE "NAVI_SIM_CAMEAR_SIM_COMPONENT_BUILDING_DLL")
target_link_libraries(camera_sim_component embree3)
rclcpp_components_register_nodes(camera_sim_component
  "navi_sim::CameraSimComponent")
ament_auto_add_executable(camera_sim_node
  src/camera_sim_node.cpp
)
target_link_libraries(camera_sim_node camera_sim_component)

ament_auto_add_library(semantic_map_sim_component SHARED
  src/semantic_map_sim_component.cpp
  src/primitives/box.cpp
  src/primitives/primitive.cpp
  src/raycaster.cpp
)
target_compile_definitions(semantic_map_sim_component PRIVATE "NAVI_SIM_SEMANTIC_MAP_SIM_COMPONENT_BUILDING_DLL")
target_link_libraries(semantic_map_sim_component embree3)
rclcpp_components_register_nodes(semantic_map_sim_component
  "navi_sim::SemanticMapSimComponent")
ament_target_dependencies(semantic_map_sim_component
  vision_msgs
)
ament_auto_add_executable(semantic_map_sim_node
  src/semantic_map_sim_node.cpp
)
ament_target_dependencies(semantic_map_sim_node
  vision_msgs
)
target_link_libraries(semantic_map_sim_node semantic_map_sim_component)

ament_auto_add_library(scenario_test_component SHARED
  src/interpreter/action_base.cpp
  src/interpreter/black_board.cpp
  src/interpreter/data_types.cpp
  src/interpreter/event_base.cpp
  src/interpreter/interpreter.cpp
  src/interpreter/reach_position_event.cpp
  src/interpreter/send_goal_action.cpp
  src/interpreter/simulation_time_event.cpp
  src/interpreter/terminate_action.cpp
  src/primitives/box.cpp
  src/primitives/primitive.cpp
  src/raycaster.cpp
  src/scenario_test_component.cpp
)
target_compile_definitions(scenario_test_component PRIVATE "NAVI_SIM_SCENARIO_TEST_COMPONENT_BUILDING_DLL")
target_link_libraries(scenario_test_component embree3 yaml-cpp)
rclcpp_components_register_nodes(scenario_test_component
  "navi_sim::ScenarioTestComponent")
ament_auto_add_executable(scenario_test_node
  src/scenario_test_node.cpp
)
target_link_libraries(scenario_test_node
  yaml-cpp
  scenario_test_component
)

ament_auto_add_executable(example
  src/interpreter/action_base.cpp
  src/interpreter/black_board.cpp
  src/interpreter/data_types.cpp
  src/interpreter/event_base.cpp
  src/interpreter/example.cpp
  src/interpreter/interpreter.cpp
  src/interpreter/reach_position_event.cpp
  src/interpreter/send_goal_action.cpp
  src/interpreter/simulation_time_event.cpp
  src/interpreter/terminate_action.cpp
)
target_link_libraries(example
  yaml-cpp
)

add_executable(check_result
  src/check_result.cpp
)
target_link_libraries(check_result
  yaml-cpp
)

install(TARGETS
  navi_sim_node
  lidar_sim_node
  camera_sim_node
  semantic_map_sim_node
  scenario_test_node
  example
  check_result
  DESTINATION lib/navi_sim
)
install(TARGETS
  navi_sim_component
  lidar_sim_component
  camera_sim_component
  semantic_map_sim_component
  scenario_test_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
ament_export_libraries(navi_sim_component)
ament_export_libraries(lidar_sim_component)
ament_export_libraries(camera_sim_component)
ament_export_libraries(semantic_map_sim_component)
ament_export_libraries(scenario_test_component)

option(DEVELOP "help string describing option" OFF)

#############
## Install ##
#############
install(DIRECTORY launch config scenarios
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_model test/src/test_model.cpp)
  target_link_libraries(test_model lidar_sim_component)
  # This testcase includes ament_index_cpp::get_package_share_directory, and it cannot be used in building debian package.
  if(DEVELOP)
    ament_add_gtest(test_interpreter test/src/test_interpreter.cpp)
    target_link_libraries(test_interpreter scenario_test_component)
  endif()
endif()

ament_auto_package()
