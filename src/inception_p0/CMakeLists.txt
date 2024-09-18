################################################################################
# Set minimum required version of cmake, project name and compile options
################################################################################
cmake_minimum_required(VERSION 3.8)
project(inception_p0)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

################################################################################
# Find ament packages and libraries for ament and system dependencies
################################################################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

################################################################################
# Install
################################################################################
install(DIRECTORY
  launch
  models
  rviz
  urdf
  map
  worlds
  params
  scripts
  bt_xml
  config
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python scripts
install(
  PROGRAMS
  scripts/red_cube_detect.py
  scripts/yolo_v2.py
  scripts/combine_obs.py
  DESTINATION lib/${PROJECT_NAME}/
)

# Install C++ behaviors
set(BEHAVIOR_SOURCES
  src/navigation_behaviors.cpp 
  src/person_node.cpp
)

set(TARGET_DEPENDS
  rclcpp
  behaviortree_cpp_v3
  yaml-cpp
  rclcpp_action
  nav2_msgs
  tf2
  tf2_geometry_msgs
)

include_directories(include)

add_executable(autonomy_node src/autonomy_node.cpp ${BEHAVIOR_SOURCES})
ament_target_dependencies(autonomy_node ${TARGET_DEPENDS})
target_link_libraries(autonomy_node ${YAML_CPP_LIBRARIES})

install(TARGETS
  autonomy_node
  DESTINATION lib/${PROJECT_NAME}/
)

# Environment hooks
ament_environment_hooks("${CMAKE_CURRENT_SOURCE_DIR}/env-hooks/multi_robot.dsv.in")

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Uncomment these lines if necessary
  # set(ament_cmake_copyright_FOUND TRUE)
  # set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
