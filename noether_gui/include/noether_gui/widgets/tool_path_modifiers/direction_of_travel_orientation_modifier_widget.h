#pragma once

#include <noether_gui/widgets.h>

#include <noether_tpp/core/tool_path_modifier.h>

namespace noether
{
struct DirectionOfTravelOrientationModifierWidget : public ToolPathModifierWidget
{
  Q_OBJECT
  using ToolPathModifierWidget::ToolPathModifierWidget;
  ToolPathModifier::ConstPtr create() const override;
};

}  // namespace noether
