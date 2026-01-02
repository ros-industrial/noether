#include <noether_gui/widgets/tool_path_modifiers/uniform_spacing_modifier_widget.h>
#include "ui_uniform_spacing_modifier_widget.h"
#include <noether_gui/widgets/distance_double_spin_box.h>

#include <noether_tpp/serialization.h>

namespace noether
{
UniformSpacingModifierWidget::UniformSpacingModifierWidget(QWidget* parent)
  : BaseWidget(parent), ui_(new Ui::UniformSpacing())
{
  ui_->setupUi(this);
}

void UniformSpacingModifierWidget::configure(const YAML::Node& config)
{
  ui_->point_spacing->setValue(YAML::getMember<double>(config, "point_spacing"));
  ui_->spline_degree->setValue(YAML::getMember<int>(config, "spline_degree"));
  ui_->include_endpoints->setChecked(YAML::getMember<bool>(config, "include_endpoints"));
}

void UniformSpacingModifierWidget::save(YAML::Node& config) const
{
  config["name"] = "UniformSpacing";
  config["point_spacing"] = ui_->point_spacing->value();
  config["spline_degree"] = ui_->spline_degree->value();
  config["include_endpoints"] = ui_->include_endpoints->isChecked();
}

}  // namespace noether
