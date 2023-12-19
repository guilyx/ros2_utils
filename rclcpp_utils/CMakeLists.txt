cmake_minimum_required(VERSION 3.5)
project(rclcpp_utils)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)

set(dependencies
  "rclcpp"
  "rclcpp_action"
)

include_directories(include)

set(LIBRARY_NAME "${PROJECT_NAME}_lib")

add_library(${LIBRARY_NAME} INTERFACE)

ament_target_dependencies(
  ${LIBRARY_NAME} INTERFACE
  ${dependencies}
)

install(TARGETS ${LIBRARY_NAME}
  EXPORT "export_${PROJECT_NAME}"
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib
  INCLUDES DESTINATION include
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
)

#ament_export_libraries(${LIBRARY_NAME} INTERFACE)
ament_export_include_directories(include)
ament_export_dependencies(${dependencies})
ament_export_targets("export_${PROJECT_NAME}")

ament_package()
