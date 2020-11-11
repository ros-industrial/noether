cmake_minimum_required(VERSION 2.8.3)
project(noether_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  geometry_msgs
  message_generation
  pcl_msgs
  shape_msgs
  std_msgs
  heat_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
    FilteringConfig.msg
    PlaneSlicerRasterGeneratorConfig.msg
    SurfaceWalkRasterGeneratorConfig.msg
    EigenValueEdgeGeneratorConfig.msg
    HalfedgeEdgeGeneratorConfig.msg
    SegmentationConfig.msg
    ToolPathConfig.msg
    ToolPath.msg
    ToolPaths.msg
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
    GenerateToolPaths.action
    ApplyMeshFilters.action
    Segment.action
)
##Generate service files in the 'srv' folder
add_service_files(
 FILES
    GenerateConvexHull.srv
)
generate_messages(
  DEPENDENCIES
    actionlib_msgs
    geometry_msgs
    pcl_msgs
    shape_msgs
    std_msgs
    heat_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
  CATKIN_DEPENDS
    actionlib_msgs
    geometry_msgs
    message_runtime
    pcl_msgs
    shape_msgs
    std_msgs
    heat_msgs
)
