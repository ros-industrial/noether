#pragma once

#include <noether_gui/widgets.h>
#include <noether_tpp/core/tool_path_modifier.h>

class QDoubleSpinBox;
class QSpinBox;

namespace noether
{
class DistanceDoubleSpinBox;

/**
 * @ingroup gui_widgets_tool_path_modifiers
 */
class CircularLeadOutToolPathModifierWidget : public ToolPathModifierWidget
{
public:
  CircularLeadOutToolPathModifierWidget(QWidget* parent = nullptr);

  ToolPathModifier::ConstPtr create() const override;

  void configure(const YAML::Node&) override;
  void save(YAML::Node&) const override;

private:
  QDoubleSpinBox* arc_angle_;
  DistanceDoubleSpinBox* arc_radius_;
  QSpinBox* n_points_;
};

}  // namespace noether
