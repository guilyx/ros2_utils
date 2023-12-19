cmake_minimum_required(VERSION 3.5)
project(example)

set(CMAKE_CXX_STANDARD 20)

# lots of warnings and all warnings
if (MSVC)
    add_compile_options(/W4)
else()
    add_compile_options(-Wall -Wextra -pedantic)
endif()

find_package(ament_cmake REQUIRED)

add_library(example SHARED
  src/example.cpp
)
target_link_libraries(example
  PUBLIC
  ${catkin_LIBRARIES}
)
target_include_directories(example PUBLIC include/)

ament_target_dependencies(example)

install(
  DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS example
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/example
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  add_subdirectory(test)
endif()

ament_export_include_directories(include)
ament_export_libraries(example)
ament_package()