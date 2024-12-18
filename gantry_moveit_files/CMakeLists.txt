cmake_minimum_required(VERSION 3.1.3)
project(gantry_moveit_files)

find_package(catkin REQUIRED COMPONENTS
  std_msgs
  rospy
  gazebo_msgs
  roscpp
  geometry_msgs
  roscpp
  interactive_markers
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  moveit_ros_perception
  rviz_visual_tools
  moveit_visual_tools
  pluginlib
  geometric_shapes
  pcl_ros
  pcl_conversions
  rosbag
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  message_generation
)
add_service_files(DIRECTORY srv FILES
  GetBoolVal.srv
)

add_message_files(
   FILES
   DimDataArray.msg
 )

generate_messages(
  DEPENDENCIES 
  std_msgs 
  geometry_msgs
)

find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED system filesystem date_time thread)



catkin_install_python(PROGRAMS src/detection.py 
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
    moveit_core
    moveit_visual_tools
    moveit_ros_planning_interface
    interactive_markers
    tf2_geometry_msgs
    roscpp
    rospy
  DEPENDS
    EIGEN3
)

include_directories(${THIS_PACKAGE_INCLUDE_DIRS})
include_directories(SYSTEM ${catkin_INCLUDE_DIRS} ${Boost_INCLUDE_DIR} ${EIGEN3_INCLUDE_DIRS})

add_executable(gantry_moveit_files src/move_group_interface_tutorial.cpp)
install(DIRECTORY config launch src msg srv DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
target_link_libraries(gantry_moveit_files ${catkin_LIBRARIES} ${Boost_LIBRARIES})
install(TARGETS gantry_moveit_files DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

