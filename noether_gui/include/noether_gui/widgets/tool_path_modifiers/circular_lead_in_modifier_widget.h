#pragma once

#include <noether_gui/widgets.h>

class QSpinBox;

namespace noether
{
class AngleDoubleSpinBox;
class DistanceDoubleSpinBox;

/**
 * @ingroup gui_widgets_tool_path_modifiers
 */
class CircularLeadInToolPathModifierWidget : public ToolPathModifierWidget
{
public:
  CircularLeadInToolPathModifierWidget(QWidget* parent = nullptr);

  void configure(const YAML::Node&) override;
  void save(YAML::Node&) const override;

private:
  AngleDoubleSpinBox* arc_angle_;
  DistanceDoubleSpinBox* arc_radius_;
  QSpinBox* n_points_;
};

}  // namespace noether
