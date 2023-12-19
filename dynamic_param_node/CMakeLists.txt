cmake_minimum_required(VERSION 3.5)
project(dynamic_param_node)

set(CMAKE_CXX_STANDARD 20)

# lots of warnings and all warnings
if (MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -pedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)

add_library(dynamic_param_node SHARED
  src/dynamic_param_node.cpp
)
target_link_libraries(dynamic_param_node
  PUBLIC
  ${catkin_LIBRARIES}
)
target_include_directories(dynamic_param_node PUBLIC include/)

ament_target_dependencies(dynamic_param_node)

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS dynamic_param_node
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/dynamic_param_node
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(dynamic_param_node)
ament_package()