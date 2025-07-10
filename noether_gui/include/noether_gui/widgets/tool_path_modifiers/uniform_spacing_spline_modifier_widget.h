#pragma once

#include <noether_gui/widgets.h>
#include <noether_tpp/core/tool_path_modifier.h>

namespace noether
{
class DistanceDoubleSpinBox;

class UniformSpacingSplineModifierWidget : public ToolPathModifierWidget
{
public:
  UniformSpacingSplineModifierWidget(QWidget* parent = nullptr);

  ToolPathModifier::ConstPtr create() const override;
  void configure(const YAML::Node& config) override;
  void save(YAML::Node& config) const override;

protected:
  DistanceDoubleSpinBox* point_spacing_;
};

}  // namespace noether
