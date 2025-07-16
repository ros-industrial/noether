#pragma once

#include <noether_gui/widgets/tool_path_planners/edge/edge_planner_widget.h>

namespace noether
{
/**
 * @ingroup gui_widgets_tool_path_planners
 */
class BoundaryEdgePlannerWidget : public EdgePlannerWidget
{
public:
  using EdgePlannerWidget::configure;
  using EdgePlannerWidget::EdgePlannerWidget;

  void save(YAML::Node&) const override;
};

}  // namespace noether
