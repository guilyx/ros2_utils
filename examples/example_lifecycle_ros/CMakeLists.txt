cmake_minimum_required(VERSION 3.5)
project(example_lifecycle_ros)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies

find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(example_msgs REQUIRED)
find_package(example REQUIRED)

include_directories(include
	${std_msgs_INCLUDE_DIRS}
  ${std_srvs_INCLUDE_DIRS}
	${rclcpp_INCLUDE_DIRS}
  ${rclcpp_lifecycle_INCLUDE_DIRS}
	${example_msgs_INCLUDE_DIRS}
  ${example_INCLUDE_DIRS}
)

add_library(example_lifecycle_ros_lib src/example_lifecycle_ros.cpp)

add_executable(example_lifecycle_ros src/main.cpp src/example_lifecycle_ros.cpp)

target_link_libraries(example_lifecycle_ros
  example_lifecycle_ros_lib
	${std_msgs_LIBRARIES}
  ${std_srvs_LIBRARIES}
  ${rclcpp_LIBRARIES}
  ${rclcpp_lifecycle_LIBRARIES}
  ${example_msgs_LIBRARIES}
  ${example_LIBRARIES}
)

set(dependencies
  "rclcpp"
  "rclcpp_lifecycle"
  "std_msgs"
  "std_srvs"
  "example_msgs"
  "example"
)

ament_target_dependencies(example_lifecycle_ros_lib
  "rclcpp"
  "rclcpp_lifecycle"
  "std_msgs"
  "std_srvs"
  "example_msgs"
  "example"
)

install(TARGETS example_lifecycle_ros DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch params DESTINATION share/${PROJECT_NAME})

ament_package()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  add_subdirectory(test)
endif()