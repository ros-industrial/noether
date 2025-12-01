#include <noether_gui/widgets/tool_path_planners/edge/boundary_edge_planner_widget.h>

namespace noether
{
void BoundaryEdgePlannerWidget::save(YAML::Node& config) const { config["name"] = "Boundary"; }

}  // namespace noether
