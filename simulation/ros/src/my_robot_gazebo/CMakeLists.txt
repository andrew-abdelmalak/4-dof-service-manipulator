cmake_minimum_required(VERSION 3.0.2)
project(my_robot_gazebo)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  gazebo_msgs
  sensor_msgs
)

catkin_package(
#  INCLUDE_DIRS INCLUDE_DIRS
#  LIBRARIES my_robot_gazebo
#  CATKIN_DEPENDS rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
