#pragma once

#include <noether_gui/widgets.h>

namespace noether
{
/**
 * @ingroup gui_widgets_tool_path_modifiers
 */
struct RasterOrganizationModifierWidget : public BaseWidget
{
public:
  using BaseWidget::BaseWidget;

  void save(YAML::Node&) const override;
};

}  // namespace noether
