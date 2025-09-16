#include <noether_gui/widgets/mesh_modifiers/fill_holes_modifier_widget.h>
#include <noether_gui/widgets/distance_double_spin_box.h>

#include <noether_tpp/serialization.h>
#include <QFormLayout>

namespace noether
{
FillHolesModifierWidget::FillHolesModifierWidget(QWidget* parent)
  : BaseWidget(parent), max_hole_size_(new DistanceDoubleSpinBox(this))
{
  auto* layout = new QFormLayout(this);

  max_hole_size_->setMinimum(0.0);
  max_hole_size_->setMaximum(1000.0);
  max_hole_size_->setValue(0.100);
  max_hole_size_->setDecimals(3);
  max_hole_size_->setSingleStep(0.010);

  layout->addRow("Max hole size", max_hole_size_);
}

void FillHolesModifierWidget::configure(const YAML::Node& config)
{
  max_hole_size_->setValue(YAML::getMember<double>(config, "max_hole_size"));
}

void FillHolesModifierWidget::save(YAML::Node& config) const
{
  config["name"] = "FillHoles";
  config["max_hole_size"] = max_hole_size_->value();
}

}  // namespace noether
