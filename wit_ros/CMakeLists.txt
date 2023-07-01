cmake_minimum_required(VERSION 2.8.3)
project(wit_ros)

find_package(catkin REQUIRED COMPONENTS
  message_generation std_msgs std_srvs
)

catkin_python_setup()

add_message_files(DIRECTORY msg FILES Entity.msg Outcome.msg)
add_service_files(DIRECTORY srv FILES Interpret.srv ListenAndInterpret.srv)

generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS rospy message_runtime std_msgs std_srvs)

catkin_install_python(PROGRAMS
  scripts/wit_ros/wit_node.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )
