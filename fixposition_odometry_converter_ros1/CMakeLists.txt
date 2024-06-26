# GENERAL ==============================================================================================================
cmake_minimum_required(VERSION 3.5)
project(fixposition_odometry_converter_ros1 LANGUAGES CXX)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic\
    -Wshadow -Wunused-parameter -Wformat -Wpointer-arith")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# DEPENDENCIES =========================================================================================================
find_package(Boost 1.65.0 REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  geometry_msgs
  fixposition_driver_ros1
)

catkin_package(
 INCLUDE_DIRS include
 CATKIN_DEPENDS
  roscpp geometry_msgs
  nav_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS}  ${EIGEN3_INCLUDE_DIR} ${Boost_INCLUDE_DIR})


# BUILD EXECUTABLE =====================================================================================================
add_executable(
	${PROJECT_NAME}
  src/odom_converter_node.cpp
  src/odom_converter.cpp
  src/params.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES})

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
