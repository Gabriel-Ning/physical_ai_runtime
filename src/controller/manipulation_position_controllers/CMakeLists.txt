cmake_minimum_required(VERSION 3.16)
project(manipulation_position_controllers VERSION 0.3.7)

add_compile_options(-Wall -Wextra -Wpedantic)

if(POLICY CMP0167)
  cmake_policy(SET CMP0167 OLD)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
  set(CMAKE_CXX_STANDARD_REQUIRED ON)
endif()

find_package(ament_cmake REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(controller_interface REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)
find_package(pinocchio REQUIRED)
find_package(placo REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)
find_package(ruckig REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)

# Keep this controller's OSQP ABI private. MoveIt brings a different libosqp
# ABI into the same process; linking either dependency by its public SONAME
# makes plugin loading depend on whichever library the loader sees first.
find_program(PATCHELF_EXECUTABLE patchelf REQUIRED)

function(get_imported_library_location output target)
  foreach(property IMPORTED_LOCATION_RELEASE IMPORTED_LOCATION)
    get_target_property(location ${target} ${property})
    if(location AND NOT location STREQUAL "location-NOTFOUND")
      set(${output} "${location}" PARENT_SCOPE)
      return()
    endif()
  endforeach()
  message(FATAL_ERROR "Could not resolve imported library location for ${target}")
endfunction()

get_imported_library_location(OSQP_LIBRARY osqp::osqp)
get_imported_library_location(OSQP_EIGEN_LIBRARY OsqpEigen::OsqpEigen)
if(TARGET qdldl::qdldl)
  get_imported_library_location(QDLDL_LIBRARY qdldl::qdldl)
else()
  find_library(QDLDL_LIBRARY NAMES qdldl REQUIRED)
endif()

execute_process(
  COMMAND nm -D ${OSQP_LIBRARY}
  OUTPUT_VARIABLE OSQP_DYNAMIC_SYMBOLS
  COMMAND_ERROR_IS_FATAL ANY)
if(NOT OSQP_DYNAMIC_SYMBOLS MATCHES "OSQPCscMatrix_set_data")
  message(FATAL_ERROR
    "The selected libosqp is not the OSQP 1.0 ABI required by OsqpEigen 0.10.3: "
    "${OSQP_LIBRARY}")
endif()

get_target_property(
  OSQP_EIGEN_SONAME OsqpEigen::OsqpEigen IMPORTED_SONAME_RELEASE)
if(NOT OSQP_EIGEN_SONAME OR OSQP_EIGEN_SONAME STREQUAL "OSQP_EIGEN_SONAME-NOTFOUND")
  get_filename_component(OSQP_EIGEN_SONAME ${OSQP_EIGEN_LIBRARY} NAME)
endif()

set(PRIVATE_OSQP_DIR ${CMAKE_CURRENT_BINARY_DIR}/private_osqp)
set(PRIVATE_OSQP_LIBRARY ${PRIVATE_OSQP_DIR}/libmpc_osqp.so)
set(PRIVATE_OSQP_EIGEN_LIBRARY ${PRIVATE_OSQP_DIR}/libmpc_osqp_eigen.so)
set(PRIVATE_QDLDL_LIBRARY ${PRIVATE_OSQP_DIR}/libmpc_qdldl.so)

add_custom_command(
  OUTPUT
    ${PRIVATE_OSQP_LIBRARY}
    ${PRIVATE_OSQP_EIGEN_LIBRARY}
    ${PRIVATE_QDLDL_LIBRARY}
  COMMAND ${CMAKE_COMMAND} -E make_directory ${PRIVATE_OSQP_DIR}
  COMMAND ${CMAKE_COMMAND} -E copy ${OSQP_LIBRARY} ${PRIVATE_OSQP_LIBRARY}
  COMMAND ${CMAKE_COMMAND} -E copy ${OSQP_EIGEN_LIBRARY} ${PRIVATE_OSQP_EIGEN_LIBRARY}
  COMMAND ${CMAKE_COMMAND} -E copy ${QDLDL_LIBRARY} ${PRIVATE_QDLDL_LIBRARY}
  COMMAND ${PATCHELF_EXECUTABLE}
    --set-soname libmpc_qdldl.so ${PRIVATE_QDLDL_LIBRARY}
  COMMAND ${PATCHELF_EXECUTABLE} --set-soname libmpc_osqp.so ${PRIVATE_OSQP_LIBRARY}
  COMMAND ${PATCHELF_EXECUTABLE}
    --replace-needed libqdldl.so libmpc_qdldl.so ${PRIVATE_OSQP_LIBRARY}
  COMMAND ${PATCHELF_EXECUTABLE} --set-rpath $ORIGIN ${PRIVATE_OSQP_LIBRARY}
  COMMAND ${PATCHELF_EXECUTABLE}
    --replace-needed libosqp.so libmpc_osqp.so ${PRIVATE_OSQP_EIGEN_LIBRARY}
  COMMAND ${PATCHELF_EXECUTABLE}
    --set-soname libmpc_osqp_eigen.so ${PRIVATE_OSQP_EIGEN_LIBRARY}
  COMMAND ${PATCHELF_EXECUTABLE} --set-rpath $ORIGIN ${PRIVATE_OSQP_EIGEN_LIBRARY}
  DEPENDS ${OSQP_LIBRARY} ${OSQP_EIGEN_LIBRARY} ${QDLDL_LIBRARY}
  VERBATIM)
add_custom_target(private_osqp_runtime ALL DEPENDS
  ${PRIVATE_OSQP_LIBRARY}
  ${PRIVATE_OSQP_EIGEN_LIBRARY}
  ${PRIVATE_QDLDL_LIBRARY})

add_library(mpc_osqp SHARED IMPORTED GLOBAL)
set_target_properties(mpc_osqp PROPERTIES
  IMPORTED_LOCATION ${PRIVATE_OSQP_LIBRARY}
  INTERFACE_INCLUDE_DIRECTORIES
    "$<TARGET_PROPERTY:osqp::osqp,INTERFACE_INCLUDE_DIRECTORIES>")

add_library(mpc_osqp_eigen SHARED IMPORTED GLOBAL)
set_target_properties(mpc_osqp_eigen PROPERTIES
  IMPORTED_LOCATION ${PRIVATE_OSQP_EIGEN_LIBRARY}
  INTERFACE_INCLUDE_DIRECTORIES
    "$<TARGET_PROPERTY:OsqpEigen::OsqpEigen,INTERFACE_INCLUDE_DIRECTORIES>"
  INTERFACE_COMPILE_DEFINITIONS
    "$<TARGET_PROPERTY:OsqpEigen::OsqpEigen,INTERFACE_COMPILE_DEFINITIONS>")

add_library(${PROJECT_NAME} SHARED
  src/common/joint_ordering.cpp
  src/common/manipulation_diagnostics.cpp
  src/joint_space/dynamics/joint_space_impedance_controller.cpp
  src/joint_space/kinematics/joint_space_position_controller.cpp
  src/joint_space/kinematics/reference_message_reader.cpp
  src/joint_space/kinematics/ruckig_adapter.cpp
  src/joint_space/kinematics/trajectory_sampler.cpp
  src/task_space/kinematics/task_space_kinematic_position_controller.cpp
  src/task_space/dynamics/task_space_joint_impedance_controller.cpp
)
add_dependencies(${PROJECT_NAME} private_osqp_runtime)
set_target_properties(${PROJECT_NAME} PROPERTIES
  BUILD_RPATH ${PRIVATE_OSQP_DIR}
  INSTALL_RPATH "$ORIGIN/${PROJECT_NAME}:$ORIGIN")

target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(${PROJECT_NAME}
  control_toolbox
  controller_interface
  diagnostic_msgs
  diagnostic_updater
  Eigen3
  geometry_msgs
  hardware_interface
  moveit_msgs
  pinocchio
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  ruckig
  sensor_msgs
  std_msgs
  trajectory_msgs
)
target_link_libraries(${PROJECT_NAME}
  pinocchio::pinocchio
  mpc_osqp
  mpc_osqp_eigen
  placo::placo
  ruckig::ruckig)

pluginlib_export_plugin_description_file(
  controller_interface manipulation_position_controllers.xml)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  FILES
    ${PRIVATE_OSQP_LIBRARY}
    ${PRIVATE_OSQP_EIGEN_LIBRARY}
    ${PRIVATE_QDLDL_LIBRARY}
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  FILES
    config/joint_space_impedance_controller.yaml
    config/joint_space_position_controller.yaml
    config/task_space_kinematic_position_controller.yaml
    config/task_space_joint_impedance_controller.yaml
  DESTINATION share/${PROJECT_NAME}/config
)

ament_export_dependencies(
  control_toolbox
  controller_interface
  diagnostic_msgs
  diagnostic_updater
  Eigen3
  geometry_msgs
  hardware_interface
  moveit_msgs
  osqp
  OsqpEigen
  pinocchio
  placo
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
  ruckig
  sensor_msgs
  std_msgs
  trajectory_msgs
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(launch_testing_ament_cmake REQUIRED)

  # Pure-function unit tests (no ROS runtime required to run).
  ament_add_gtest(test_trajectory_sampler test/test_trajectory_sampler.cpp)
  target_link_libraries(test_trajectory_sampler ${PROJECT_NAME})

  ament_add_gtest(test_reference_message_reader test/test_reference_message_reader.cpp)
  target_link_libraries(test_reference_message_reader ${PROJECT_NAME})
  ament_target_dependencies(test_reference_message_reader rclcpp trajectory_msgs)

  ament_add_gtest(test_joint_ordering test/test_joint_ordering.cpp)
  target_link_libraries(test_joint_ordering ${PROJECT_NAME})

  ament_add_gtest(test_manipulation_diagnostics
    test/test_manipulation_diagnostics.cpp)
  target_link_libraries(test_manipulation_diagnostics ${PROJECT_NAME})

  ament_add_gtest(test_stale_guard test/test_stale_guard.cpp)
  target_link_libraries(test_stale_guard ${PROJECT_NAME})
  ament_target_dependencies(test_stale_guard rclcpp)

  ament_add_gtest(test_ruckig_adapter test/test_ruckig_adapter.cpp)
  target_link_libraries(test_ruckig_adapter ${PROJECT_NAME})

  # Pure pose-chunk helpers (chunk_is_fresh / sample_pose_chunk).
  ament_add_gtest(test_pose_chunk_buffer test/test_pose_chunk_buffer.cpp)
  target_link_libraries(test_pose_chunk_buffer ${PROJECT_NAME})

  ament_add_gtest(test_cartesian_trajectory_reader
    test/test_cartesian_trajectory_reader.cpp)
  target_link_libraries(test_cartesian_trajectory_reader ${PROJECT_NAME})
  ament_target_dependencies(test_cartesian_trajectory_reader moveit_msgs)

  # DLS IK solver: frame consistency (non-root base_frame), IK direction, and
  # damped null-space posture leakage bound. Needs Pinocchio + Eigen.
  ament_add_gtest(test_dls_solver test/test_dls_solver.cpp)
  target_link_libraries(test_dls_solver ${PROJECT_NAME} pinocchio::pinocchio)
  ament_target_dependencies(test_dls_solver pinocchio Eigen3)

  # OSQP IK solver: timestep integration (DLS-parity step) + stationary-target
  # convergence (no limit cycle). Needs Pinocchio + OSQP.
  ament_add_gtest(test_osqp_solver test/test_osqp_solver.cpp)
  add_dependencies(test_osqp_solver private_osqp_runtime)
  set_target_properties(test_osqp_solver PROPERTIES
    BUILD_RPATH ${PRIVATE_OSQP_DIR})
  target_link_libraries(test_osqp_solver ${PROJECT_NAME}
    pinocchio::pinocchio mpc_osqp mpc_osqp_eigen)
  ament_target_dependencies(test_osqp_solver pinocchio Eigen3)

  ament_add_gtest(test_placo_solver test/test_placo_solver.cpp)
  target_link_libraries(test_placo_solver ${PROJECT_NAME} placo::placo)
  ament_target_dependencies(test_placo_solver Eigen3)

  ament_add_gtest(test_joint_space_impedance_controller test/test_joint_space_impedance_controller.cpp)
  target_link_libraries(test_joint_space_impedance_controller ${PROJECT_NAME})

  ament_add_gtest(test_task_space_joint_impedance_controller
    test/test_task_space_joint_impedance_controller.cpp)
  target_link_libraries(test_task_space_joint_impedance_controller ${PROJECT_NAME})

  ament_add_gtest(test_controller_lifecycle test/test_controller_lifecycle.cpp)
  target_link_libraries(test_controller_lifecycle ${PROJECT_NAME})
  ament_target_dependencies(test_controller_lifecycle
    controller_interface hardware_interface rclcpp rclcpp_lifecycle)

  add_launch_test(
    test/test_jspc_fake_hardware.py
    TIMEOUT 45
  )
endif()

ament_package()
