cmake_minimum_required(VERSION 3.0.2)
project(hand_detection)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES hand_detection
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

# include_directories(
# # include
# # ${catkin_INCLUDE_DIRS}
# )

catkin_install_python(PROGRAMS
   scripts/hand_detection/hand_detection_node.py
  #  scripts/yolo.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )