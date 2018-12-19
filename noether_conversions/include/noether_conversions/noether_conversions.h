#include <geometry_msgs/PoseArray.h>

#include <tool_path_planner/tool_path_planner_base.h>

namespace noether {

  std::vector<geometry_msgs::PoseArray> convertVTKtoGeometryMsgs(
      const std::vector<tool_path_planner::ProcessPath>& paths);

}

