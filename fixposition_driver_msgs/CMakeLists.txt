# GENERAL ==============================================================================================================

cmake_minimum_required(VERSION 3.16)
project(fixposition_driver_msgs VERSION 8.0.0 LANGUAGES CXX C)

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

# This gives us ROS_VERSION
find_package(ros_environment REQUIRED)
set(ROS_VERSION $ENV{ROS_VERSION})

find_package(Eigen3 REQUIRED)
find_package(fpsdk_common REQUIRED)
find_package(fixposition_driver_lib REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)


# BUILD, INSTALL =======================================================================================================

# --- ROS 1 ------------------------------------------------------------------------------------------------------------
if(${ROS_VERSION} EQUAL 1)

    find_package(catkin REQUIRED COMPONENTS
        #message_generation
        std_msgs
        geometry_msgs
        nav_msgs
        sensor_msgs
    )

    file(GLOB MSG_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR}/msg msg/*.msg)
    add_message_files(FILES ${MSG_FILES})

    generate_messages(DEPENDENCIES
        std_msgs geometry_msgs nav_msgs sensor_msgs
    )

    catkin_package(
        INCLUDE_DIRS include
        LIBRARIES ${PROJECT_NAME}
        CATKIN_DEPENDS
            #message_runtime
            std_msgs geometry_msgs nav_msgs sensor_msgs
    )

    include_directories(
        include
        ${catkin_INCLUDE_DIRS}
        ${fixposition_driver_lib_INCLUDE_DIRS}
        ${fpsdk_common_INLCUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
    )

    add_library(${PROJECT_NAME} SHARED
        src/data_to_ros.cpp
    )

    target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${fixposition_driver_lib_LIBRARIES}
        ${fpsdk_common_LIBRARIES}
    )

    install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

    install(TARGETS ${PROJECT_NAME}
        #EXPORT ${PROJECT_NAME}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

# --- ROS 2 ------------------------------------------------------------------------------------------------------------
elseif(${ROS_VERSION} EQUAL 2)

    # https://answers.ros.org/question/357633/

    find_package(ament_cmake REQUIRED)
    find_package(builtin_interfaces REQUIRED)
    find_package(rosidl_default_generators REQUIRED)
    find_package(rclcpp REQUIRED)

    file(GLOB MSG_FILES RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} msg/*.msg)
    rosidl_generate_interfaces(${PROJECT_NAME} ADD_LINTER_TESTS
        ${MSG_FILES}
        DEPENDENCIES
            builtin_interfaces
            std_msgs geometry_msgs nav_msgs sensor_msgs
    )

    ament_export_dependencies(rosidl_default_runtime)

    rosidl_get_typesupport_target(cpp_typesupport_target "${PROJECT_NAME}" "rosidl_typesupport_cpp")

    add_library(${PROJECT_NAME}_lib SHARED
        src/data_to_ros.cpp
    )
    target_include_directories(${PROJECT_NAME}_lib PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
        ${fixposition_driver_lib_INCLUDE_DIRS}
        ${fpsdk_common_INLCUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
    )

    target_link_libraries(${PROJECT_NAME}_lib PUBLIC
        ${cpp_typesupport_target}
        ${fixposition_driver_lib_LIBRARIES}
        ${fpsdk_common_LIBRARIES}
        ${fpsdk_common_LIBRARIES}
    )

    install(DIRECTORY include/
        DESTINATION include/${PROJECT_NAME}
    )

    install(
        TARGETS ${PROJECT_NAME}_lib
        EXPORT export_${PROJECT_NAME}_lib
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin
    )

    ament_export_include_directories(include)
    ament_export_libraries(${PROJECT_NAME}_lib)
    ament_export_targets(export_${PROJECT_NAME}_lib)

    ament_package()

endif()
