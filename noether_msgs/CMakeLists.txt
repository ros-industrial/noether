cmake_minimum_required(VERSION 2.8.3)
project(noether_msgs)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  geometry_msgs
  shape_msgs
  std_msgs
  pcl_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  ToolPathConfig.msg
  ToolRasterPath.msg
)

## Generate actions in the 'action' folder
add_action_files(
  FILES
  GenerateToolPaths.action
  Segment.action
)

generate_messages(
  DEPENDENCIES
    actionlib_msgs
    geometry_msgs
    shape_msgs
    std_msgs
    pcl_msgs
)

###################################
## catkin specific configuration ##
###################################
catkin_package(
 CATKIN_DEPENDS message_runtime geometry_msgs shape_msgs std_msgs
)
