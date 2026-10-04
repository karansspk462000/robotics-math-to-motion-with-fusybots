cmake_minimum_required(VERSION 3.8)
project(kinematics)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(math_utils REQUIRED)

add_library(${PROJECT_NAME}
  src/context/kinematics.cpp
  src/context/forward_kinematics.cpp
  src/context/inverse_kinematics.cpp
  src/common/jacobian.cpp
  src/strategy/analytical_six_axis_forward_kinematics.cpp
  src/strategy/analytical_six_axis_inverse_kinematics.cpp
  src/strategy/kdl_forward_kinematics.cpp
  src/strategy/kdl_inverse_kinematics.cpp
  src/factory/kinematics_factory.cpp
)
target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${orocos_kdl_INCLUDE_DIRS}
)
target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
  yaml-cpp::yaml-cpp
  ${orocos_kdl_LIBRARIES}
  math_utils::math_utils
)
ament_target_dependencies(${PROJECT_NAME} PUBLIC kdl_parser ament_index_cpp)

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
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 yaml-cpp orocos_kdl kdl_parser ament_index_cpp math_utils)

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
