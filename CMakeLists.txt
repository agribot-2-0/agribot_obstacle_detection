cmake_minimum_required(VERSION 2.8.3)
project(agribot_obstacle_detection)
add_compile_options(-std=c++17)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rviz_visual_tools
  pcl_ros
  pcl_conversions
  sensor_msgs
  agribot_msgs
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
)

find_package(Eigen3 REQUIRED)

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
                                              include/agribot_obstacle_detection/lidar/LidarObstacleDetectionNode.h
                                              include/agribot_obstacle_detection/lidar/cloud_processor.h
                                              include/agribot_obstacle_detection/lidar/object.h
                                              include/agribot_obstacle_detection/lidar/bounding_box.h
                                              )
target_link_libraries(lidar_obstacle_detection_node ${catkin_LIBRARIES})
add_dependencies(lidar_obstacle_detection_node ${catkin_EXPORTED_TARGETS})


# Sonar Obstacle Detection Node
add_executable(sonar_obstacle_detection_node  src/sonar/sonar_obstacle_detection_node.cpp
                                              src/sonar/SonarObstacleDetectionNode.cpp
                                              src/sonar/sonar.cpp
                                              )
target_link_libraries(sonar_obstacle_detection_node ${catkin_LIBRARIES})
add_dependencies(sonar_obstacle_detection_node ${catkin_EXPORTED_TARGETS})

# Obstacle Arbiter Node
add_executable(obstacle_arbiter_node  src/arbiter/obstacle_arbiter_node.cpp
                                      src/arbiter/ObstacleArbiterNode.cpp
                                      src/arbiter/sat_collision_detection/sat.cpp
                                      src/arbiter/sat_collision_detection/Projection.cpp
                                      )
target_link_libraries(obstacle_arbiter_node ${catkin_LIBRARIES})
add_dependencies(obstacle_arbiter_node ${catkin_EXPORTED_TARGETS})

# SAT test
add_executable(sat_test  test/arbiter/sat_test.cpp
                         src/arbiter/sat_collision_detection/sat.cpp
                         src/arbiter/sat_collision_detection/Projection.cpp
                         )
target_link_libraries(sat_test ${catkin_LIBRARIES})
add_dependencies(sat_test ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############

install(TARGETS
  lidar_obstacle_detection_node
  sonar_obstacle_detection_node
  obstacle_arbiter_node
  sat_test
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch param bags scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

catkin_install_python(PROGRAMS
   scripts/obstacle_detection_observer_app.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############
