# GENERAL ==============================================================================================================
cmake_minimum_required(VERSION 3.5)
project(fixposition_driver_ros2)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic\
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
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(fixposition_driver_lib REQUIRED)
find_package(rtcm_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  msg/Speed.msg
  msg/Gnsssats.msg
  msg/NMEA.msg
  msg/WheelSensor.msg
  msg/fpa/GNSSANT.msg
  msg/fpa/GNSSCORR.msg
  msg/fpa/IMUBIAS.msg
  msg/fpa/EOE.msg
  msg/fpa/LLH.msg
  msg/fpa/ODOMENU.msg
  msg/fpa/ODOMETRY.msg
  msg/fpa/ODOMSH.msg
  msg/fpa/ODOMSTATUS.msg
  msg/fpa/TEXT.msg
  msg/fpa/TP.msg
  msg/nmea/GPGGA.msg
  msg/nmea/GPGLL.msg
  msg/nmea/GNGSA.msg
  msg/nmea/GPGST.msg
  msg/nmea/GXGSV.msg
  msg/nmea/GPHDT.msg
  msg/nmea/GPRMC.msg
  msg/nmea/GPVTG.msg
  msg/nmea/GPZDA.msg
  msg/extras/COVWARN.msg
  DEPENDENCIES
  std_msgs
  nav_msgs
  geometry_msgs
  builtin_interfaces
)
ament_export_dependencies(rosidl_default_runtime)

include_directories(
  include
  ${sensor_msgs_INCLUDE_DIR}
  ${fixposition_driver_lib_INCLUDE_DIR}
  ${rtcm_msgs_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIR}
)

add_executable(
  ${PROJECT_NAME}_exec
  src/fixposition_driver_node.cpp
  src/params.cpp
  src/data_to_ros2.cpp
)

if($ENV{ROS_DISTRO} MATCHES "humble|jazzy|rolling")
  rosidl_get_typesupport_target(
    cpp_typesupport_target
    ${PROJECT_NAME} "rosidl_typesupport_cpp"
  )
else()
  message(FATAL_ERROR "Unsupported ROS_DISTRO")
endif()

target_link_libraries(
  ${PROJECT_NAME}_exec
  ${fixposition_driver_lib_LIBRARIES}
  ${rtcm_msgs_LIBRARIES}
  ${Boost_LIBRARIES}
  ${EIGEN3_LIBRARIES}
  ${cpp_typesupport_target}
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
ament_target_dependencies(${PROJECT_NAME}_exec rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_eigen fixposition_driver_lib rtcm_msgs tf2_geometry_msgs)

# define ament package for this project
ament_package()
