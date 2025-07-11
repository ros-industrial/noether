#pragma once

#include <noether_gui/widgets.h>

#include <noether_tpp/core/tool_path_modifier.h>

namespace noether
{
/**
 * @ingroup gui_widgets_tool_path_modifiers
 */
struct SnakeOrganizationModifierWidget : public ToolPathModifierWidget
{
public:
  using ToolPathModifierWidget::ToolPathModifierWidget;

  ToolPathModifier::ConstPtr create() const override;
};

}  // namespace noether
