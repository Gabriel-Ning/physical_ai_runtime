cmake_minimum_required(VERSION 3.16)
project(execution_manager VERSION 0.2.0)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(action_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(execution_manager_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(yaml-cpp REQUIRED)

add_library(execution_manager_core STATIC
  src/profile.cpp
  src/router.cpp
  src/selector.cpp
)
target_include_directories(execution_manager_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)
target_link_libraries(execution_manager_core PUBLIC yaml-cpp)
ament_target_dependencies(execution_manager_core PUBLIC
  action_msgs
  diagnostic_msgs
  execution_manager_interfaces
  geometry_msgs
  rclcpp
  std_msgs
  trajectory_msgs
)

add_executable(execution_manager_node
  src/main.cpp
  src/node.cpp
)
target_link_libraries(execution_manager_node
  execution_manager_core
)
ament_target_dependencies(execution_manager_node
  action_msgs
  control_msgs
  controller_manager_msgs
  diagnostic_msgs
  execution_manager_interfaces
  geometry_msgs
  moveit_msgs
  rcl_interfaces
  rclcpp
  rclcpp_action
  std_msgs
  trajectory_msgs
)

# Also create execution_manager executable alias for compatibility
add_executable(execution_manager
  src/main.cpp
  src/node.cpp
)
target_link_libraries(execution_manager
  execution_manager_core
)
ament_target_dependencies(execution_manager
  action_msgs
  control_msgs
  controller_manager_msgs
  diagnostic_msgs
  execution_manager_interfaces
  geometry_msgs
  moveit_msgs
  rcl_interfaces
  rclcpp
  rclcpp_action
  std_msgs
  trajectory_msgs
)

install(TARGETS execution_manager_core execution_manager_node execution_manager
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include/${PROJECT_NAME})
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_execution_manager_core test/test_execution_manager_core.cpp)
  target_link_libraries(test_execution_manager_core execution_manager_core)
  ament_target_dependencies(test_execution_manager_core execution_manager_interfaces)
  target_compile_definitions(test_execution_manager_core PRIVATE
    WORKSPACE_ROOT="${CMAKE_CURRENT_SOURCE_DIR}/../../.."
  )

  ament_add_gtest(test_action_proxy
    test/test_action_proxy.cpp
    src/node.cpp
  )
  target_link_libraries(test_action_proxy execution_manager_core)
  ament_target_dependencies(test_action_proxy
    action_msgs
    control_msgs
    controller_manager_msgs
    diagnostic_msgs
    execution_manager_interfaces
    geometry_msgs
    moveit_msgs
    rcl_interfaces
    rclcpp
    rclcpp_action
    std_msgs
    trajectory_msgs
  )
  target_compile_definitions(test_action_proxy PRIVATE
    WORKSPACE_ROOT="${CMAKE_CURRENT_SOURCE_DIR}/../../.."
  )
  set_tests_properties(test_action_proxy PROPERTIES ENVIRONMENT
    "ROS_DOMAIN_ID=97;ROS_LOG_DIR=/tmp/execution_manager_test_logs"
  )

  ament_add_gtest(test_external_source_single_flight
    test/test_external_source_single_flight.cpp
    src/node.cpp
  )
  target_link_libraries(test_external_source_single_flight execution_manager_core)
  ament_target_dependencies(test_external_source_single_flight
    action_msgs
    control_msgs
    controller_manager_msgs
    diagnostic_msgs
    execution_manager_interfaces
    geometry_msgs
    moveit_msgs
    rcl_interfaces
    rclcpp
    rclcpp_action
    std_msgs
    trajectory_msgs
  )
  target_compile_definitions(test_external_source_single_flight PRIVATE
    WORKSPACE_ROOT="${CMAKE_CURRENT_SOURCE_DIR}/../../.."
  )
  set_tests_properties(test_external_source_single_flight PROPERTIES
    ENVIRONMENT
      "ROS_DOMAIN_ID=98;ROS_LOG_DIR=/tmp/execution_manager_test_logs"
  )
endif()

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(execution_manager_core)
ament_export_targets(export_${PROJECT_NAME})
ament_export_dependencies(
  action_msgs
  control_msgs
  controller_manager_msgs
  diagnostic_msgs
  execution_manager_interfaces
  geometry_msgs
  moveit_msgs
  rcl_interfaces
  rclcpp
  rclcpp_action
  std_msgs
  trajectory_msgs
)
ament_package()
