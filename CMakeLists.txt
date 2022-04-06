# GENERAL ==============================================================================================================
cmake_minimum_required(VERSION 3.5)
project(fixposition_driver LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fdiagnostics-color=always -Wall -Wextra -Wpedantic -Wno-unused-parameter")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# DEPENDENCIES =========================================================================================================
find_package(Boost 1.65.0 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  tf
  nav_msgs
  sensor_msgs
  std_msgs
  geometry_msgs
  message_generation
  fixposition_gnss_tf
)

add_message_files(
    FILES
    VRTK.msg
    Speed.msg
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
  tf
  roscpp geometry_msgs
  nav_msgs std_msgs message_runtime
)

include_directories(include ${catkin_INCLUDE_DIRS}  ${EIGEN3_INCLUDE_DIR} ${Boost_INCLUDE_DIR})


# BUILD EXECUTABLE =====================================================================================================
add_executable(
	${PROJECT_NAME}
  src/fixposition_driver_node.cpp
  src/fixposition_driver.cpp
  src/base_converter.cpp
  src/odometry.cpp
  src/llh.cpp
  src/imu.cpp
  src/tf.cpp
  src/helper.cpp
  src/params.cpp
  
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} pthread)
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

