#pragma once

#include <noether_gui/widgets/tool_path_modifiers/linear_approach_modifier_widget.h>

namespace noether
{
/**
 * @ingroup gui_widgets_tool_path_modifiers
 */
class LinearDepartureToolPathModifierWidget : public LinearApproachToolPathModifierWidget
{
public:
  using LinearApproachToolPathModifierWidget::LinearApproachToolPathModifierWidget;

  void save(YAML::Node&) const override;
};

}  // namespace noether
