cmake_minimum_required(VERSION 3.0.2)
project(lab4_jetauto_control)

find_package(catkin REQUIRED COMPONENTS
  rospy
  geometry_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/jetauto_square_control.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
