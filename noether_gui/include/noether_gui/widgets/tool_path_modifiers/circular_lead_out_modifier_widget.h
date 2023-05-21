#pragma once

#include <noether_gui/widgets.h>
#include <noether_tpp/core/tool_path_modifier.h>

class QDoubleSpinBox;
class QSpinBox;

namespace noether
{
class CircularLeadOutToolPathModifierWidget : public ToolPathModifierWidget
{
  Q_OBJECT
public:
  CircularLeadOutToolPathModifierWidget(QWidget* parent = nullptr);

  ToolPathModifier::ConstPtr create() const override;

  void configure(const YAML::Node&) override;
  void save(YAML::Node&) const override;

private:
  QDoubleSpinBox* arc_angle_;
  QDoubleSpinBox* arc_radius_;
  QSpinBox* n_points_;
};

}  // namespace noether
