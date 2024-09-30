#pragma once

#include <noether_gui/widgets.h>
#include <noether_tpp/core/tool_path_modifier.h>

class QDoubleSpinBox;

namespace noether
{
class SplineModifierWidget : public ToolPathModifierWidget
{
public:
  SplineModifierWidget(QWidget* parent = nullptr);

  ToolPathModifier::ConstPtr create() const override;
  void configure(const YAML::Node& config) override;
  void save(YAML::Node& config) const override;

protected:
  QDoubleSpinBox* point_spacing_;
};

}  // namespace noether
