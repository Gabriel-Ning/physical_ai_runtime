cmake_minimum_required(VERSION 3.16)
project(piper_leader_teleop VERSION 0.1.0 LANGUAGES CXX)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(piper_description REQUIRED)
find_package(libpiper CONFIG REQUIRED)

add_executable(piper_leader_node src/piper_leader_node.cpp)
ament_target_dependencies(piper_leader_node
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
  trajectory_msgs
)
target_link_libraries(piper_leader_node libpiper::libpiper)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_launch_contract test/test_launch_contract.py)
endif()

install(TARGETS piper_leader_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})

ament_package()
