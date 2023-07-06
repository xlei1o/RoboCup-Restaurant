cmake_minimum_required(VERSION 3.0.2)
project(object_clustering)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_geometry
  image_transport
  roscpp
  std_msgs
  darknet_ros_msgs
  tf
  tf_conversions
  objects_msgs
  message_generation
)

find_package(OpenCV REQUIRED)
find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES object_clustering
  CATKIN_DEPENDS cv_bridge image_geometry image_transport objects_msgs roscpp std_msgs darknet_ros_msgs tf tf_conversions
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/object_clustering.cpp
)

add_executable(${PROJECT_NAME}_node 
  src/applications/object_clustering_node.cpp
)


target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  ${PCL_LIBRARIES}
)

target_link_libraries(${PROJECT_NAME}_node
  ${catkin_LIBRARIES}
  object_clustering
)
