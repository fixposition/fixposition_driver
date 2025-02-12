# GENERAL ==============================================================================================================

cmake_minimum_required(VERSION 3.16)
project(fixposition_driver_ros1 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic -Werror\
    -Wshadow -Wunused-parameter -Wformat -Wpointer-arith -Woverloaded-virtual")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()
if(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
    add_compile_definitions(NDEBUG)
endif()


# DEPENDENCIES =========================================================================================================

find_package(Boost 1.65.0 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fixposition_driver_lib REQUIRED)
find_package(fixposition_driver_msgs REQUIRED)
find_package(fpsdk_common REQUIRED)
find_package(fpsdk_ros1 REQUIRED)
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
  ${fixposition_driver_msgs_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
  ${Boost_INCLUDE_DIR}
  ${fpsdk_common_INLCUDE_DIRS} ${fpsdk_ros1_INLCUDE_DIRS}
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
  ${fpsdk_common_LIBRARIES} ${fpsdk_ros1_LIBRARIES}
  pthread
)


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
