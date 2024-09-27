#include <noether_gui/widgets/tool_path_planners/edge/half_edge_planner_widget.h>

#include <noether_tpp/tool_path_planners/edge/half_edge_planner.h>

namespace noether
{
ToolPathPlanner::ConstPtr HalfEdgePlannerWidget::create() const { return std::make_unique<HalfEdgePlanner>(); }

}  // namespace noether
