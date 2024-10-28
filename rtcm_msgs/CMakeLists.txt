cmake_minimum_required(VERSION 3.5)
project(rtcm_msgs)

# ros_environment is needed to have ROS_VERSION
find_package(ros_environment REQUIRED)
set(ROS_VERSION $ENV{ROS_VERSION})

find_package(std_msgs REQUIRED)

if(${ROS_VERSION} EQUAL 1)

  find_package(catkin REQUIRED
    COMPONENTS
      message_generation
  )

  add_message_files(DIRECTORY msg
    FILES
      Message.msg
  )

  generate_messages(DEPENDENCIES std_msgs)

  catkin_package(CATKIN_DEPENDS message_runtime std_msgs)

elseif(${ROS_VERSION} EQUAL 2)
  find_package(ament_cmake REQUIRED)
  find_package(builtin_interfaces REQUIRED)
  find_package(rosidl_default_generators REQUIRED)

  rosidl_generate_interfaces(${PROJECT_NAME} ADD_LINTER_TESTS
    msg/Message.msg
    DEPENDENCIES builtin_interfaces std_msgs
  )

  ament_export_dependencies(rosidl_default_runtime)
  ament_package()

endif()
