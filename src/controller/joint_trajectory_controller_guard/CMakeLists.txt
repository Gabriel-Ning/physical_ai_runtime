cmake_minimum_required(VERSION 3.16)
project(joint_trajectory_controller_guard)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(jtc_guard_node src/jtc_guard_node.cpp)
target_include_directories(jtc_guard_node PRIVATE include)
ament_target_dependencies(jtc_guard_node
  action_msgs
  control_msgs
  diagnostic_msgs
  diagnostic_updater
  rclcpp
  rclcpp_action
  std_msgs)

install(TARGETS jtc_guard_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)
  ament_add_gtest(test_guard_core test/test_guard_core.cpp)
  target_include_directories(test_guard_core PRIVATE include)

  add_launch_test(
    test/test_jtc_disconnect.py
    TIMEOUT 45
  )
endif()

ament_package()
