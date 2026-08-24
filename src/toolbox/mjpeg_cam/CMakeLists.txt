cmake_minimum_required(VERSION 3.16)
project(mjpeg_cam)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(mjpeg_cam_node
  src/main.cpp
  src/node.cpp
)
target_compile_features(mjpeg_cam_node PUBLIC cxx_std_17)
target_include_directories(mjpeg_cam_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
)
target_link_libraries(mjpeg_cam_node
  rclcpp::rclcpp
  ${sensor_msgs_TARGETS}
)

install(TARGETS mjpeg_cam_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_jpeg test/test_jpeg.cpp)
  target_include_directories(test_jpeg PRIVATE include)
  target_compile_features(test_jpeg PUBLIC cxx_std_17)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_config_template test/test_config_template.py)
endif()

ament_package()
