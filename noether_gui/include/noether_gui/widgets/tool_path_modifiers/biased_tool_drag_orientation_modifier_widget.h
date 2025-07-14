#pragma once

#include <noether_gui/widgets.h>

#include <noether_tpp/core/tool_path_modifier.h>

class QDoubleSpinBox;

namespace noether
{
class DistanceDoubleSpinBox;

/**
 * @ingroup gui_widgets_tool_path_modifiers
 */
class BiasedToolDragOrientationToolPathModifierWidget : public ToolPathModifierWidget
{
public:
  BiasedToolDragOrientationToolPathModifierWidget(QWidget* parent = nullptr);

  ToolPathModifier::ConstPtr create() const override;

  void configure(const YAML::Node&) override;
  void save(YAML::Node&) const override;

private:
  QDoubleSpinBox* angle_offset_;
  DistanceDoubleSpinBox* tool_radius_;
};

}  // namespace noether
