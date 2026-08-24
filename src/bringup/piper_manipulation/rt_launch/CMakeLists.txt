cmake_minimum_required(VERSION 3.16)
project(piper_manipulation_rt_launch)

find_package(ament_cmake REQUIRED)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_piper_bringup_contract test/test_piper_bringup_contract.py)
endif()

install(DIRECTORY
  launch
  config
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
