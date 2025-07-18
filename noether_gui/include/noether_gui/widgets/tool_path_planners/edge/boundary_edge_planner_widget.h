#pragma once

#include <noether_gui/widgets.h>

namespace noether
{
/**
 * @ingroup gui_widgets_tool_path_planners
 */
class BoundaryEdgePlannerWidget : public BaseWidget
{
public:
  using BaseWidget::BaseWidget;
  using BaseWidget::configure;

  void save(YAML::Node&) const override;
};

}  // namespace noether
