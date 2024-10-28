# GENERAL ==============================================================================================================
cmake_minimum_required(VERSION 3.5)
project(fixposition_driver_ros1 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic\
    -Wshadow -Wunused-parameter -Wformat -Wpointer-arith")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# DEPENDENCIES =========================================================================================================
find_package(Boost 1.65.0 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fixposition_driver_lib REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  nav_msgs
  sensor_msgs
  std_msgs
  geometry_msgs
  message_generation
  eigen_conversions
  tf2_ros
  tf2_geometry_msgs
)
add_message_files(
  FILES
  Speed.msg
  WheelSensor.msg
  GnssSats.msg
  NMEA.msg
  fpa/eoe.msg
  fpa/gnssant.msg
  fpa/gnsscorr.msg
  fpa/imubias.msg
  fpa/llh.msg
  fpa/odomenu.msg
  fpa/odometry.msg
  fpa/odomsh.msg
  fpa/odomstatus.msg
  fpa/text.msg
  fpa/tp.msg
  nmea/gpgga.msg
  nmea/gpgll.msg
  nmea/gngsa.msg
  nmea/gpgst.msg
  nmea/gxgsv.msg
  nmea/gphdt.msg
  nmea/gprmc.msg
  nmea/gpvtg.msg
  nmea/gpzda.msg
  extras/CovWarn.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  nav_msgs
  geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS
  tf tf2_ros
  roscpp geometry_msgs tf2_geometry_msgs
  nav_msgs std_msgs message_runtime
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${fixposition_driver_lib_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIR}
)

# BUILD EXECUTABLE =====================================================================================================
add_executable(
  ${PROJECT_NAME}
  src/fixposition_driver_node.cpp
  src/params.cpp
  src/data_to_ros1.cpp
)

target_link_libraries(
  ${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${fixposition_driver_lib_LIBRARIES}
  ${Boost_LIBRARIES}
  pthread
)

add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)

# INSTALL ==============================================================================================================
install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  "launch"
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
