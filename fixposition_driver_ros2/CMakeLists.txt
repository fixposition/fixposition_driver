# GENERAL ==============================================================================================================
cmake_minimum_required(VERSION 3.5)
project(fixposition_driver_ros2)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Werror\
    -Wshadow -Wunused-parameter -Wformat -Wpointer-arith")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

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
find_package(rclcpp_lifecycle REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(diagnostic_updater REQUIRED)
find_package(fixposition_driver_lib REQUIRED)
find_package(fixposition_driver_msgs REQUIRED)
find_package(rtcm_msgs REQUIRED)
find_package(fpsdk_common REQUIRED)
find_package(fpsdk_ros2 REQUIRED)
find_package(rosbag2_cpp REQUIRED)

include_directories(
  include
  ${sensor_msgs_INCLUDE_DIR}
  ${fixposition_driver_lib_INCLUDE_DIRS}
  ${fixposition_driver_msgs_INCLUDE_DIRS}
  ${rtcm_msgs_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIR}
  ${fpsdk_common_INLCUDE_DIRS} ${fpsdk_ros2_INLCUDE_DIRS}
)

add_executable(
  ${PROJECT_NAME}_exec
  src/fixposition_driver_main.cpp
  src/fixposition_driver_node.cpp
  src/params.cpp
  src/data_to_ros2.cpp
)

target_link_libraries(
  ${PROJECT_NAME}_exec
  ${fixposition_driver_lib_LIBRARIES}
  ${rtcm_msgs_LIBRARIES}
  ${Boost_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${fpsdk_common_LIBRARIES} ${fpsdk_ros2_LIBRARIES}
  ${rosbag2_cpp_TARGETS}
  pthread
)

add_executable(
  ${PROJECT_NAME}_lifecycle_exec
  src/fixposition_driver_lifecycle_node.cpp
  src/fixposition_driver_node.cpp
  src/params.cpp
  src/data_to_ros2.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_lifecycle_exec
  ${fixposition_driver_lib_LIBRARIES}
  ${rtcm_msgs_LIBRARIES}
  ${Boost_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${fpsdk_common_LIBRARIES} ${fpsdk_ros2_LIBRARIES}
  ${rosbag2_cpp_TARGETS}
  pthread
)

install(DIRECTORY include/
  DESTINATION .
)

install(TARGETS ${PROJECT_NAME}_exec ${PROJECT_NAME}_lifecycle_exec
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  "launch"
  DESTINATION share/${PROJECT_NAME}/
)
ament_target_dependencies(${PROJECT_NAME}_exec
    rclcpp rclcpp_lifecycle std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_eigen
    diagnostic_msgs diagnostic_updater fixposition_driver_lib fixposition_driver_msgs rtcm_msgs tf2_geometry_msgs)
ament_target_dependencies(${PROJECT_NAME}_lifecycle_exec
    rclcpp rclcpp_lifecycle std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_eigen
    diagnostic_msgs diagnostic_updater fixposition_driver_lib fixposition_driver_msgs rtcm_msgs tf2_geometry_msgs)

# define ament package for this project
ament_package()
