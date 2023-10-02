cmake_minimum_required(VERSION 2.8.3)
project(radiolink)

find_package(catkin REQUIRED COMPONENTS rospy geometry_msgs sensor_msgs)

catkin_package()

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

catkin_install_python(PROGRAMS
	nodes/link.py
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/nodes
)
