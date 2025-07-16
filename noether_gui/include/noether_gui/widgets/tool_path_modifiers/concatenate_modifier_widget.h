#pragma once

#include <noether_gui/widgets.h>

namespace noether
{
/**
 * @ingroup gui_widgets_tool_path_modifiers
 */
struct ConcatenateModifierWidget : public ToolPathModifierWidget
{
public:
  using ToolPathModifierWidget::ToolPathModifierWidget;

  void save(YAML::Node&) const override;
};

}  // namespace noether
