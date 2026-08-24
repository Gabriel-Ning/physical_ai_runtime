cmake_minimum_required(VERSION 3.16)
project(piper_manipulation_workstation_launch)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(
    test_execution_manager_recording_integration
    test/test_execution_manager_recording_integration.py
    TIMEOUT 60
  )
  ament_add_pytest_test(
    test_piper_workstation_bringup_contract
    test/test_piper_workstation_bringup_contract.py
  )
endif()

get_filename_component(REPO_ROOT "${CMAKE_CURRENT_SOURCE_DIR}/../../../.." ABSOLUTE)
install(DIRECTORY "${REPO_ROOT}/apps/profiles/"
  DESTINATION share/${PROJECT_NAME}/profiles
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
