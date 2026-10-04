cmake_minimum_required(VERSION 3.8)
project(hw_interface)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(custom_robot_msgs REQUIRED)
find_package(robot_config REQUIRED)
find_package(math_utils REQUIRED)
find_package(robot_hw_interface REQUIRED)
find_package(digital_twin REQUIRED)

add_library(${PROJECT_NAME}
  src/strategy/dummy_hardware_interface.cpp
  src/strategy/arduino_hardware_interface.cpp
  src/factory/hw_interface_factory.cpp
  src/hw_interface_node.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC rclcpp custom_robot_msgs)
target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
  yaml-cpp::yaml-cpp
  robot_config::robot_config
  math_utils::math_utils
  robot_hw_interface::robot_hw_interface
)

add_executable(hw_interface_node src/hw_interface_node_main.cpp)
target_link_libraries(hw_interface_node PUBLIC ${PROJECT_NAME} digital_twin::digital_twin)
ament_target_dependencies(hw_interface_node PUBLIC ament_index_cpp)

install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(
  TARGETS hw_interface_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp custom_robot_msgs Eigen3 yaml-cpp robot_config math_utils robot_hw_interface)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
