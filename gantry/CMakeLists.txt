cmake_minimum_required(VERSION 3.21.3)

project(gantry)

find_package(catkin REQUIRED COMPONENTS
  rospy
  gazebo_msgs
  roscpp
  geometry_msgs
)

catkin_package()

find_package(roslaunch)

foreach(dir config launch meshes urdf)
	install(DIRECTORY ${dir}/
		DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/${dir})
endforeach(dir)
