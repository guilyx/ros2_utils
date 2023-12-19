cmake_minimum_required(VERSION 3.5)
project(example_ros)

set(CMAKE_CXX_STANDARD 20)

# lots of warnings and all warnings
if (MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -pedantic)
endif()

# find dependencies

# ament
find_package(ament_cmake REQUIRED)

# general dependencies
find_package(example REQUIRED)

# ros dependencies
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(example_msgs REQUIRED)

add_library(example_ros_lib
  src/example_ros.cpp
)
target_link_libraries(example_ros_lib)
target_include_directories(example_ros_lib PUBLIC include)

add_executable(example_ros src/main.cpp)
target_link_libraries(example_ros example_ros_lib)

set(dependencies
  "rclcpp"
  "std_msgs"
  "std_srvs"
  "example_msgs"
  "swarm_launch_utils"
  "example"
)

ament_target_dependencies(example_ros_lib
  ${dependencies}
)

install(TARGETS example_ros DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch params DESTINATION share/${PROJECT_NAME})

ament_package()

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  add_subdirectory(test)
endif()