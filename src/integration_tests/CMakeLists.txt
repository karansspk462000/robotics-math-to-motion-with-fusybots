cmake_minimum_required(VERSION 3.8)
project(integration_tests)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(kinematics REQUIRED)
find_package(motion_planner REQUIRED)
find_package(path_planner REQUIRED)
find_package(path_interpolator REQUIRED)
find_package(trajectory_planner REQUIRED)
find_package(state_orchestrator REQUIRED)
find_package(collision_checking REQUIRED)

if(BUILD_TESTING)
  # One test binary per architectural area, instead of one file for the
  # whole pipeline - kinematics/trajectory_planner tests only link what they
  # actually exercise, and motion_planner/robot_state_machine each get their
  # own file so a failure immediately says which layer broke.
  ament_add_gtest(test_kinematics test/test_kinematics.cpp)
  target_link_libraries(test_kinematics kinematics::kinematics)

  ament_add_gtest(test_trajectory_planner test/test_trajectory_planner.cpp)
  target_link_libraries(test_trajectory_planner trajectory_planner::trajectory_planner)

  ament_add_gtest(test_path_planner test/test_path_planner.cpp)
  target_link_libraries(test_path_planner path_planner::path_planner)

  ament_add_gtest(test_collision_checking test/test_collision_checking.cpp)
  target_link_libraries(test_collision_checking collision_checking::collision_checking kinematics::kinematics)

  ament_add_gtest(test_motion_planner test/test_motion_planner.cpp)
  target_link_libraries(test_motion_planner
    kinematics::kinematics
    motion_planner::motion_planner
    path_planner::path_planner
    path_interpolator::path_interpolator
    trajectory_planner::trajectory_planner
  )

  ament_add_gtest(test_robot_state_machine test/test_robot_state_machine.cpp)
  target_link_libraries(test_robot_state_machine state_orchestrator::state_orchestrator)
endif()

ament_package()
