cmake_minimum_required(VERSION 3.0.2)
project(cs4023_project1)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
)

# Install Python scripts
catkin_install_python(PROGRAMS
  scripts/my_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
