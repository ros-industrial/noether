#include <noether_gui/widgets/tool_path_modifiers/uniform_spacing_linear_modifier_widget.h>
#include "ui_uniform_spacing_linear_modifier_widget.h"
#include <noether_gui/widgets/distance_double_spin_box.h>

#include <noether_tpp/serialization.h>

namespace noether
{
UniformSpacingLinearModifierWidget::UniformSpacingLinearModifierWidget(QWidget* parent)
  : BaseWidget(parent), ui_(new Ui::UniformSpacing())
{
  ui_->setupUi(this);
}

void UniformSpacingLinearModifierWidget::configure(const YAML::Node& config)
{
  ui_->point_spacing->setValue(YAML::getMember<double>(config, "point_spacing"));
  if (config["spline_degree"])
    ui_->spline_degree->setValue(YAML::getMember<int>(config, "spline_degree"));
  else
    ui_->spline_degree->setValue(1);
}

void UniformSpacingLinearModifierWidget::save(YAML::Node& config) const
{
  config["name"] = "UniformSpacingLinear";
  config["point_spacing"] = ui_->point_spacing->value();
  config["spline_degree"] = ui_->spline_degree->value();
}

}  // namespace noether
