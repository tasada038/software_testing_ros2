cmake_minimum_required(VERSION 3.5)
project(software_testing_ros2)

# Find Dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

# Include Directories
include_directories(
  ${PROJECT_SOURCE_DIR}/include
)

# Build libraries for publisher and subscriber components
add_library(publisher_component
  src/publisher_component.cpp
)
ament_target_dependencies(publisher_component
  rclcpp
  std_msgs
)

add_library(subscriber_component
  src/subscriber_component.cpp
)
ament_target_dependencies(subscriber_component
  rclcpp
  std_msgs
)

# Create executables for publisher and subscriber nodes
add_executable(publisher_node
  src/publisher_node.cpp
)
target_link_libraries(publisher_node
  publisher_component
)
ament_target_dependencies(publisher_node
  rclcpp
  std_msgs
)

add_executable(subscriber_node
  src/subscriber_node.cpp
)
target_link_libraries(subscriber_node
  subscriber_component
)
ament_target_dependencies(subscriber_node
  rclcpp
  std_msgs
)

# ----- Software Testing Process ----- #
option(BUILD_UNIT_TEST "Build unit tests" OFF)
option(BUILD_COMPONENT_TEST "Build component tests" OFF)
option(BUILD_INTEGRATION_TEST "Build integration tests" OFF)

# Unit Testing
if(BUILD_UNIT_TEST)
  ament_add_gtest(unit_test_publisher test/unit_test_publisher_component.cpp)
  target_link_libraries(unit_test_publisher
    publisher_component
  )
  ament_target_dependencies(unit_test_publisher
    rclcpp
    std_msgs
  )
endif()


# Component Testing
if(BUILD_COMPONENT_TEST)
  ament_add_gtest(comp_test_publisher test/component_test_publisher_component.cpp)
  target_link_libraries(comp_test_publisher
    publisher_component
  )
  ament_target_dependencies(comp_test_publisher
    rclcpp
    std_msgs
  )
endif()


# Integration Testing
if(BUILD_INTEGRATION_TEST)
  ament_add_gtest(integration_test test/integration_test_publisher_component.cpp)
  target_link_libraries(integration_test
    publisher_component
    subscriber_component
  )
  ament_target_dependencies(integration_test
    rclcpp
    std_msgs
  )
endif()

install(TARGETS
  publisher_node
  subscriber_node
  DESTINATION lib/${PROJECT_NAME})

# Add compiler options for coverage
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} --coverage")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} --coverage")
  endif()
endif()

# Custom target for coverage report
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_custom_target(
    coverage
    COMMAND ${CMAKE_COMMAND} -E make_directory ${CMAKE_BINARY_DIR}/coverage_html_report
    # Capture coverage data
    COMMAND lcov --directory ${CMAKE_BINARY_DIR} --capture --output-file ${CMAKE_BINARY_DIR}/coverage.info
    # Remove unwanted files from coverage (system and external libraries)
    COMMAND lcov --remove ${CMAKE_BINARY_DIR}/coverage.info '/usr/*' '/opt/ros/*' '*/gtest/*' '*/test/*' --output-file ${CMAKE_BINARY_DIR}/coverage_filtered.info
    # Generate HTML report
    COMMAND genhtml ${CMAKE_BINARY_DIR}/coverage_filtered.info --output-directory ${CMAKE_BINARY_DIR}/coverage_html_report
    WORKING_DIRECTORY ${CMAKE_BINARY_DIR}
    COMMENT "Generating coverage report"
  )
endif()


ament_package()
