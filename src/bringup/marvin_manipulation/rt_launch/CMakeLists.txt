cmake_minimum_required(VERSION 3.16)
project(marvin_manipulation_rt_launch)

find_package(ament_cmake REQUIRED)
find_package(libmarvin CONFIG REQUIRED)

add_executable(clear_errors src/clear_errors.cpp)
target_compile_definitions(clear_errors PRIVATE CMPL_LIN)
target_link_libraries(clear_errors PRIVATE libmarvin::libmarvin)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_marvin_bringup_contract test/test_marvin_bringup_contract.py)
endif()

install(TARGETS clear_errors
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

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
