cmake_minimum_required(VERSION 3.8)
project(state_orchestrator)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(custom_robot_msgs REQUIRED)
find_package(robot_config REQUIRED)
find_package(math_utils REQUIRED)
find_package(kinematics REQUIRED)
find_package(path_planner REQUIRED)
find_package(path_interpolator REQUIRED)
find_package(trajectory_planner REQUIRED)
find_package(motion_planner REQUIRED)
find_package(collision_checking REQUIRED)

add_library(${PROJECT_NAME}
  src/robot_node.cpp
  src/state_manager/robot_state.cpp
  src/state_manager/robot_state_machine.cpp
  src/state_manager/states/robot_states.cpp
  src/visualization/joint_state_bridge_node.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC rclcpp custom_robot_msgs sensor_msgs)
target_link_libraries(${PROJECT_NAME} PUBLIC
  robot_config::robot_config
  math_utils::math_utils
  kinematics::kinematics
  path_planner::path_planner
  path_interpolator::path_interpolator
  trajectory_planner::trajectory_planner
  motion_planner::motion_planner
)

add_executable(robot_node src/robot_node_main.cpp)
target_link_libraries(robot_node PUBLIC ${PROJECT_NAME} collision_checking::collision_checking)
ament_target_dependencies(robot_node PUBLIC ament_index_cpp)

add_executable(joint_state_bridge_node src/visualization/joint_state_bridge_node_main.cpp)
target_link_libraries(joint_state_bridge_node PUBLIC ${PROJECT_NAME})
ament_target_dependencies(joint_state_bridge_node PUBLIC ament_index_cpp)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(
  DIRECTORY rviz/
  DESTINATION share/${PROJECT_NAME}/rviz
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS robot_node joint_state_bridge_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp sensor_msgs custom_robot_msgs robot_config math_utils kinematics path_planner path_interpolator trajectory_planner motion_planner)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
