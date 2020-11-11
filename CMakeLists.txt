cmake_minimum_required (VERSION 2.8.1)

########+ Project settings ########
PROJECT(fixposition_output)
set(CMAKE_CXX_STANDARD 14)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  nav_msgs
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
  CATKIN_DEPENDS 
  roscpp 
  nav_msgs std_msgs 
)

######## Build and include settings ########
include_directories(include ${catkin_INCLUDE_DIRS})
get_property(dirs DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY INCLUDE_DIRECTORIES)
foreach(dir ${dirs})
  message(STATUS "dir='${dir}'")
endforeach()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fpermissive") #-mfp16-format=ieee -pedantic

add_executable(
	fixposition_output
  src/fixposition_output_node.cpp
  src/fixposition_output.cpp
)

target_link_libraries(fixposition_output ${catkin_LIBRARIES} pthread)

install(DIRECTORY include/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
## Mark executables and/or libraries for installation
install(TARGETS fixposition_output
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

install(DIRECTORY
  "launch"
  "config"
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

