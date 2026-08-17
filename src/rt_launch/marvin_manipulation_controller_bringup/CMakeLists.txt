cmake_minimum_required(VERSION 3.16)
project(marvin_manipulation_controller_bringup)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_server_contract test/test_server_contract.py)
endif()

install(DIRECTORY
  launch
  config
  urdf
  docs
  DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)

install(FILES
  package.xml
  README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
