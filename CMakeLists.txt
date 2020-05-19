cmake_minimum_required(VERSION 2.8.3)
project(agribot_obstacle_detection)
add_compile_options(-std=c++14)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz_visual_tools
)

find_package(PCL 1.10 REQUIRED)
add_definitions(${PCL_DEFINITIONS})

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
 ${PCL_LIBRARY_DIRS}
)

add_executable(lidar_obstacle_detection_node  src/lidar_obstacle_detection_node.cpp
                                              include/agribot_obstacle_detection/cloud_processor.h
                                              include/agribot_obstacle_detection/object.h
                                              include/agribot_obstacle_detection/bounding_box.h
                                              )
target_link_libraries(lidar_obstacle_detection_node ${catkin_LIBRARIES} ${PCL_LIBRARIES})
add_dependencies(lidar_obstacle_detection_node ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############

#############
## Testing ##
#############
