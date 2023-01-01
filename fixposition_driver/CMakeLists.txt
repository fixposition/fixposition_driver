# GENERAL ==============================================================================================================
cmake_minimum_required(VERSION 3.5)
project(fixposition_driver)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fdiagnostics-color=always -Wall -Wextra -Wpedantic -Wno-unused-parameter")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# DEPENDENCIES =========================================================================================================
find_package(Boost 1.65.0 REQUIRED)
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
find_package(fixposition_gnss_tf REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
    msg/VRTK.msg
    msg/Speed.msg
    DEPENDENCIES
    std_msgs
    nav_msgs
    geometry_msgs
    builtin_interfaces
)
ament_export_dependencies(rosidl_default_runtime)

include_directories(include ${EIGEN3_INCLUDE_DIR} ${Boost_INCLUDE_DIR})

add_executable(
  ${PROJECT_NAME}_exec
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
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} "rosidl_typesupport_cpp")

target_link_libraries(${PROJECT_NAME}_exec ${fixposition_gnss_tf_LIBRARIES} ${Boost_LIBRARIES} ${EIGEN3_LIBRARIES} ${cpp_typesupport_target} pthread)
#add_dependencies(${PROJECT_NAME}  ${PROJECT_NAME}_generate_messages_cpp)

install(DIRECTORY include/
	DESTINATION .}
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
ament_target_dependencies(${PROJECT_NAME}_exec rclcpp std_msgs nav_msgs geometry_msgs sensor_msgs tf2_ros fixposition_gnss_tf)
# define ament package for this project
ament_package()


