#pragma once

#include <noether_gui/widgets.h>

#include <noether_tpp/core/tool_path_modifier.h>

class QDoubleSpinBox;

namespace noether
{
class DistanceDoubleSpinBox;

class ToolDragOrientationToolPathModifierWidget : public ToolPathModifierWidget
{
public:
  ToolDragOrientationToolPathModifierWidget(QWidget* parent = nullptr);

  ToolPathModifier::ConstPtr create() const override;

  void configure(const YAML::Node&) override;
  void save(YAML::Node&) const override;

private:
  QDoubleSpinBox* angle_offset_;
  DistanceDoubleSpinBox* tool_radius_;
};

}  // namespace noether
