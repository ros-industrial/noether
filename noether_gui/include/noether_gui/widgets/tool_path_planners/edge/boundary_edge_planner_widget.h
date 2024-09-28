#pragma once

#include <noether_gui/widgets/tool_path_planners/edge/edge_planner_widget.h>

namespace noether
{
class BoundaryEdgePlannerWidget : public EdgePlannerWidget
{
public:
  using EdgePlannerWidget::configure;
  using EdgePlannerWidget::EdgePlannerWidget;
  using EdgePlannerWidget::save;

  ToolPathPlanner::ConstPtr create() const override;
};

}  // namespace noether
