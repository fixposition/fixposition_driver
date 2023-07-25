# GENERAL ==============================================================================================================
cmake_minimum_required(VERSION 3.10.2)
project(fixposition_odometry_converter_ros2)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fdiagnostics-color=always -Wall -Wextra -Wpedantic -Wno-unused-parameter")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# DEPENDENCIES =========================================================================================================
set(PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_components
  nav_msgs
  geometry_msgs
  fixposition_driver_ros2
)

find_package(ament_cmake REQUIRED)

foreach(Dependency IN ITEMS ${PACKAGE_INCLUDE_DEPENDS})
  find_package(${Dependency} REQUIRED)
endforeach()

include_directories(
  include
)

# BUILD EXECUTABLE =====================================================================================================
add_library(odom_converter SHARED 
  src/params.cpp
  src/odom_converter_node.cpp)
ament_target_dependencies(odom_converter ${PACKAGE_INCLUDE_DEPENDS})
rclcpp_components_register_nodes(odom_converter "fixposition::OdomConverterNode")
rclcpp_components_register_node(odom_converter
  PLUGIN "fixposition::OdomConverterNode"
  EXECUTABLE odom_converter_node
)

# INSTALL ==============================================================================================================
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(
  TARGETS odom_converter
  LIBRARY DESTINATION lib
)

install(DIRECTORY include
  DESTINATION .
)

ament_package()