cmake_minimum_required(VERSION 2.8.3)
project(command_processor)

find_package(catkin REQUIRED COMPONENTS
    rospy
    std_msgs
    sensor_msgs
    message_generation
    message_runtime
    cyphal_bridge
  )

catkin_python_setup()

add_service_files(FILES
    DriveState.srv
)
 generate_messages(
   DEPENDENCIES
   std_msgs
)

catkin_package(
  LIBRARIES command_processor
  CATKIN_DEPENDS message_runtime
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
	nodes/processor.py
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/nodes
)
