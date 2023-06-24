# cmake_minimum_required(VERSION 2.8.3)
# project(restaurant)

# find_package(catkin REQUIRED COMPONENTS
# hand_detection
# )

# catkin_python_setup()

# catkin_package()

# install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
# install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})


# catkin_install_python(PROGRAMS
#   scripts/restaurant/restaurant.py
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

# if (CATKIN_ENABLE_TESTING)
#   find_package(roslaunch REQUIRED)
#   roslaunch_add_file_check(launch USE_TEST_DEPENDENCIES)
# endif()


cmake_minimum_required(VERSION 3.0.2)
project(restaurant)

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
  scripts/restaurant/run.py
  #  scripts/yolo.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )