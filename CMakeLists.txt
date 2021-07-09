cmake_minimum_required(VERSION 2.8.3)
project(agribot_obstacle_detection)
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  agribot_common
  agribot_msgs
  dynamic_reconfigure
  nav_msgs
  pcl_conversions
  pcl_ros
  roscpp
  rviz_visual_tools
  sensor_msgs
  tf2
  tf2_geometry_msgs
  tf2_sensor_msgs
  tf2_ros
  eigen_conversions
)

generate_dynamic_reconfigure_options(
  cfg/LidarPointCloudPipeline.cfg
  cfg/StereoCameraPointCloudPipeline.cfg
  cfg/PointCloudLateFusion.cfg
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES agribot_obstacle_detection
 CATKIN_DEPENDS nav_msgs roscpp
#  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(lidar_point_cloud_pipeline_node 
  src/point_cloud_pipeline/lidar_point_cloud_pipeline_node.cpp
  src/point_cloud_pipeline/PointCloudPipelineNode.cpp
  src/util/ProcessingTimer.cpp
  src/util/Conversions.cpp
)
target_link_libraries(lidar_point_cloud_pipeline_node ${catkin_LIBRARIES})
add_dependencies(lidar_point_cloud_pipeline_node ${catkin_EXPORTED_TARGETS})


add_executable(stereo_camera_point_cloud_pipeline_node 
  src/point_cloud_pipeline/stereo_camera_point_cloud_pipeline_node.cpp
  src/point_cloud_pipeline/PointCloudPipelineNode.cpp
  src/util/ProcessingTimer.cpp
  src/util/Conversions.cpp
)
target_link_libraries(stereo_camera_point_cloud_pipeline_node ${catkin_LIBRARIES})
add_dependencies(stereo_camera_point_cloud_pipeline_node ${catkin_EXPORTED_TARGETS})


add_executable(object_3d_array_visualization_node
  src/rviz_markers_visualization/object_3d_array_visualization_node.cpp
  src/rviz_markers_visualization/Object3DArrayVisualizationNode.cpp
)
target_link_libraries(object_3d_array_visualization_node ${catkin_LIBRARIES})
add_dependencies(object_3d_array_visualization_node ${catkin_EXPORTED_TARGETS})

add_executable(object_polygon_array_visualization_node
  src/rviz_markers_visualization/object_polygon_array_visualization_node.cpp
  src/rviz_markers_visualization/ObjectPolygonArrayVisualizationNode.cpp
)
target_link_libraries(object_polygon_array_visualization_node ${catkin_LIBRARIES})
add_dependencies(object_polygon_array_visualization_node ${catkin_EXPORTED_TARGETS})


add_executable(point_cloud_late_fusion_node
  src/point_cloud_late_fusion/point_cloud_late_fusion_node.cpp
  src/point_cloud_late_fusion/PointCloudLateFusionNode.cpp
  src/util/ProcessingTimer.cpp
)
target_link_libraries(point_cloud_late_fusion_node ${catkin_LIBRARIES})
add_dependencies(point_cloud_late_fusion_node ${catkin_EXPORTED_TARGETS})


add_executable(point_cloud_rgb_camera_fusion_node
  src/point_cloud_rgb_camera_fusion/point_cloud_rgb_camera_fusion_node.cpp
  src/point_cloud_rgb_camera_fusion/PointCloudRgbCameraFusionNode.cpp
  src/util/ProcessingTimer.cpp
)
target_link_libraries(point_cloud_rgb_camera_fusion_node ${catkin_LIBRARIES})
add_dependencies(point_cloud_rgb_camera_fusion_node ${catkin_EXPORTED_TARGETS})


add_executable(obstacles_arbiter_node
  src/obstacles_arbiter/obstacles_arbiter_node.cpp
  src/obstacles_arbiter/ObstaclesArbiterNode.cpp
  src/obstacles_arbiter/collision_detection/SatPolygonCollisionDetection.cpp
  src/obstacles_arbiter/collision_detection/Projection.cpp
  src/util/ProcessingTimer.cpp
  src/util/Conversions.cpp
)
target_link_libraries(obstacles_arbiter_node ${catkin_LIBRARIES})
add_dependencies(obstacles_arbiter_node ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############

install(TARGETS
  lidar_point_cloud_pipeline_node
  stereo_camera_point_cloud_pipeline_node
  object_3d_array_visualization_node
  object_polygon_array_visualization_node
  point_cloud_late_fusion_node
  point_cloud_rgb_camera_fusion_node
  obstacles_arbiter_node
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY launch bags scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

catkin_install_python(PROGRAMS
   scripts/obstacle_detection_observer_app/obstacle_detection_observer_app.py
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

#############
## Testing ##
#############
