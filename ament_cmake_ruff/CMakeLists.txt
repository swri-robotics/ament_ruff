cmake_minimum_required(VERSION 3.14)
project(ament_cmake_ruff VERSION 0.0.1)

include(cmake/ament_ruff.cmake)

# find dependencies
find_package(ament_cmake_core REQUIRED)
find_package(ament_cmake_test REQUIRED)

install(DIRECTORY cmake DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_copyright REQUIRED)
  ament_copyright()

  ament_ruff()

  find_package(ament_cmake_xmllint REQUIRED)
  ament_xmllint()
endif()

ament_package(CONFIG_EXTRAS "ament_cmake_ruff-extras.cmake")
