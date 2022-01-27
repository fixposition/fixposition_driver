cmake_minimum_required (VERSION 2.8.1)

########+ Project settings ########
PROJECT(fixposition_driver)
set(CMAKE_CXX_STANDARD 14)

find_package(Boost 1.65.0 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  sensor_msgs
  std_msgs
  geometry_msgs
  message_generation
)

add_message_files(
    FILES
    VRTK.msg
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
  roscpp geometry_msgs
  nav_msgs std_msgs message_runtime
)

######## Build and include settings ########
include_directories(include ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR})
get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "dir='${dir}'")
endforeach()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive") #-mfp16-format=ieee -pedantic

add_executable(
	${PROJECT_NAME}
  src/fixposition_node.cpp
  src/fixposition_driver.cpp
  src/base_converter.cpp
  src/fp_converter.cpp
  src/llh_converter.cpp
  src/helper.cpp
)

target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} ${Boost_LIBRARIES} pthread)
add_dependencies(${PROJECT_NAME} ${PROJECT_NAME}_generate_messages_cpp)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(DIRECTORY
  "launch"
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

