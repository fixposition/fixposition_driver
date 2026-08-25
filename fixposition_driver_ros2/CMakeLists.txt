# GENERAL ==============================================================================================================
cmake_minimum_required(VERSION 3.16)
project(fixposition_driver_ros2)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Werror\
    -Wshadow -Wunused-parameter -Wformat -Wpointer-arith")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(_unused "${FPSDK_BUILD_TESTING}") # suppress warning

# DEPENDENCIES =========================================================================================================
find_package(Boost 1.65.0 REQUIRED COMPONENTS date_time)
find_package(Eigen3 REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(fixposition_driver_lib REQUIRED)
find_package(fixposition_driver_msgs REQUIRED)
find_package(rtcm_msgs REQUIRED)
find_package(fpsdk_common REQUIRED)
find_package(rosbag2_cpp REQUIRED)
find_package(Threads REQUIRED)

include_directories(
  include
  ${sensor_msgs_INCLUDE_DIR}
  ${fixposition_driver_lib_INCLUDE_DIRS}
  ${fixposition_driver_msgs_INCLUDE_DIRS}
  ${rtcm_msgs_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIR}
  ${fpsdk_common_INLCUDE_DIRS}
)

add_executable(
  ${PROJECT_NAME}_exec
  src/fixposition_driver_node.cpp
  src/params.cpp
  src/data_to_ros2.cpp
)

target_link_libraries(${PROJECT_NAME}_exec
  ${fixposition_driver_lib_LIBRARIES}
  ${rtcm_msgs_LIBRARIES}
  ${Boost_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${fpsdk_common_LIBRARIES}
  ${rosbag2_cpp_TARGETS}
  ${rclcpp_TARGETS}
  ${std_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${tf2_ros_TARGETS}
  ${tf2_eigen_TARGETS}
  ${fixposition_driver_lib_TARGETS}
  ${fixposition_driver_msgs_TARGETS}
  ${rtcm_msgs_TARGETS}
  ${tf2_geometry_msgs_TARGETS}
  pthread
)

install(DIRECTORY include/
  DESTINATION .
)

install(TARGETS ${PROJECT_NAME}_exec
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  "launch"
  DESTINATION share/${PROJECT_NAME}/
)

# define ament package for this project
ament_package()
