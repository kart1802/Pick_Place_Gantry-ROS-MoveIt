cmake_minimum_required(VERSION 3.1.3)
project(gantry_moveit_files)

find_package(catkin REQUIRED COMPONENTS
  rospy
  gazebo_msgs
  roscpp
  geometry_msgs
)

catkin_package()

install(DIRECTORY launch src DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
