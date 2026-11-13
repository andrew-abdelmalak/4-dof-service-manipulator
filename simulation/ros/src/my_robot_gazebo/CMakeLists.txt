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

catkin_install_python(PROGRAMS
  scripts/ee_pose_from_gazebo.py
  scripts/fk_validation_node.py
  scripts/forward_kinematics.py
  scripts/ik_validation_node.py
  scripts/kinematic_joint_driver.py
  scripts/kinematics.py
  scripts/trajectory_node.py
  scripts/trajectory_planner.py
  scripts/validate_roundtrip.py
  scripts/velocity_path_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
