cmake_minimum_required(VERSION 2.8.3)
project(agribot_obstacle_detection)
add_compile_options(-std=c++14)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz_visual_tools
  pcl_ros
  pcl_conversions
  sensor_msgs
  agribot_msgs
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES agribot_obstacle_detection
#  CATKIN_DEPENDS roscpp std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  include/agribot_obstacle_detection
  ${catkin_INCLUDE_DIRS}
)

link_directories(
)

# Lidar Obstacle Detection Node
add_executable(lidar_obstacle_detection_node  src/lidar/lidar_obstacle_detection_node.cpp
                                              include/agribot_obstacle_detection/lidar/cloud_processor.h
                                              include/agribot_obstacle_detection/lidar/object.h
                                              include/agribot_obstacle_detection/lidar/bounding_box.h
                                              )
target_link_libraries(lidar_obstacle_detection_node ${catkin_LIBRARIES})
add_dependencies(lidar_obstacle_detection_node ${catkin_EXPORTED_TARGETS})


# Sonar Obstacle Detection Node
add_executable(sonar_obstacle_detection_node  src/sonar/sonar_obstacle_detection_node.cpp
                                              src/sonar/sonar.cpp
                                              )
target_link_libraries(sonar_obstacle_detection_node ${catkin_LIBRARIES})
add_dependencies(sonar_obstacle_detection_node ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############

install(TARGETS
  lidar_obstacle_detection_node
  sonar_obstacle_detection_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch param bags
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)


#############
## Testing ##
#############
