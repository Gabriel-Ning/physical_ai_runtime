cmake_minimum_required(VERSION 3.16)
project(franka_manipulation_workstation_launch)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_franka_workstation_bringup_contract
    test/test_franka_workstation_bringup_contract.py
  )
endif()

# Install a copy of apps/profiles for deployed workspaces.
# Editable source of truth remains apps/profiles/.
get_filename_component(REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../.." ABSOLUTE)
install(DIRECTORY "${REPO_ROOT}/apps/profiles/"
  DESTINATION share/${PROJECT_NAME}/profiles
)
install(FILES
  "${REPO_ROOT}/apps/recording/franka_manipulation.yaml"
  "${REPO_ROOT}/apps/recording/franka_manipulation_no_cam.yaml"
  DESTINATION share/${PROJECT_NAME}/recording
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

install(FILES
  package.xml
  README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
