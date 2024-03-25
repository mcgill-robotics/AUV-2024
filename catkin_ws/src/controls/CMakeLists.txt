cmake_minimum_required(VERSION 3.0.2)
project(controls)

set(MSG_DEP_SET auv_msgs geometry_msgs std_msgs)

find_package(catkin REQUIRED COMPONENTS
	${MSG_DEP_SET}
	actionlib
    joy
    joy_teleop
	pid
	rospy
	tf
	tf2
	tf2_geometry_msgs
	tf2_ros
)

catkin_package(	CATKIN_DEPENDS 
	${MSG_DEP_SET}
	actionlib
    joy
    joy_teleop
	pid
	rospy
	tf
	tf2
	tf2_geometry_msgs
	tf2_ros
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${roscpp_INCLUDE_DIRS}
  ${tf2_ros_INCLUDE_DIRS}
  ${tf2_geometry_msgs_INCLUDE_DIRS}
  ${tf2_INCLUDE_DIRS}
)

add_executable(quaternion_pid
  src/quaternion_pid.cpp
)


target_link_libraries(
  quaternion_pid
  ${roscpp_LIBRARIES} 
  ${tf2_ros_LIBRARIES}
  ${tf2_LIBRARIES}
  ${tf2_geometry_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
)
