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
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
    FilteringConfig.msg
    SegmentationConfig.msg
    ToolPathConfig.msg
    ToolRasterPath.msg
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
    GenerateToolPaths.action
    ApplyMeshFilters.action
    Segment.action
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    geometry_msgs
    pcl_msgs
    shape_msgs
    std_msgs
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
)
