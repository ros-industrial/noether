#include <noether_gui/widgets/tool_path_planners/edge/boundary_edge_planner_widget.h>

#include <noether_tpp/tool_path_planners/edge/boundary_edge_planner.h>

namespace noether
{
ToolPathPlanner::ConstPtr BoundaryEdgePlannerWidget::create() const { return std::make_unique<BoundaryEdgePlanner>(); }

}  // namespace noether
