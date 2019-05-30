cmake_minimum_required(VERSION 3.5)
project(poly_exploration_ros)

set(CMAKE_BUILD_TYPE Debug)

set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-g -ggdb3 -Wall -Wall -Wextra -Wpedantic -lprofiler)
endif()

find_package(ament_cmake REQUIRED)
find_package(class_loader REQUIRED)
find_package(rclcpp REQUIRED)
find_package(message_filters REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(poly_exploration REQUIRED)

include_directories(include)

#add_library(poly-exploration-node SHARED
#            src/PolygonExplorerNode.cpp)
#target_compile_definitions(poly-exploration-ros
#  PRIVATE "MINIMAL_COMPOSITION_DLL")
#ament_target_dependencies(poly-exploration-ros rclcpp std_msgs class_loader)

# This package installs libraries without exporting them.
# Export the library path to ensure that the installed libraries are available.
if(NOT WIN32)
  ament_environment_hooks(
    "${ament_cmake_package_templates_ENVIRONMENT_HOOK_LIBRARY_PATH}"
    )
endif()

ament_python_install_package(scripts/)

add_executable(polygon_exploration_ros src/main.cpp src/PolygonExplorerNode.cpp)
target_link_libraries(polygon_exploration_ros poly_exploration::poly_exploration)
ament_target_dependencies(polygon_exploration_ros
  poly_exploration::poly_exploration class_loader rclcpp rclcpp_components message_filters sensor_msgs geometry_msgs nav_msgs visualization_msgs)
#add_dependencies(polygon_exploration_ros ${catkin_EXPORTED_TARGETS} poly_exploration)

install(TARGETS
  polygon_exploration_ros
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  polygon_exploration_ros
  DESTINATION lib/${PROJECT_NAME})

install(PROGRAMS
  scripts/test_publisher.py
  DESTINATION bin
)

ament_package()
