#include <geometry_msgs/PoseArray.h>
#include <tool_path_planner/raster_tool_path_planner.h>

std::vector<geometry_msgs::PoseArray> posesconvertVTKtoGeometryMsgs(const std::vector<tool_path_planner::ProcessPath>& paths);

