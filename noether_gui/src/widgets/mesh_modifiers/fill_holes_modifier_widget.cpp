#include <noether_gui/widgets/mesh_modifiers/fill_holes_modifier_widget.h>
#include <noether_gui/utils.h>

#include <noether_tpp/mesh_modifiers/fill_holes_modifier.h>
#include <QDoubleSpinBox>
#include <QFormLayout>

namespace noether
{
FillHolesModifierWidget::FillHolesModifierWidget(QWidget* parent)
  : MeshModifierWidget(parent), max_hole_size_(new QDoubleSpinBox(this))
{
  auto* layout = new QFormLayout(this);

  max_hole_size_->setMinimum(0.0);
  max_hole_size_->setMaximum(1000.0);
  max_hole_size_->setValue(0.100);
  max_hole_size_->setDecimals(3);
  max_hole_size_->setSingleStep(0.010);

  layout->addRow("Max hole size (m)", max_hole_size_);
}

MeshModifier::ConstPtr FillHolesModifierWidget::create() const
{
  return std::make_unique<FillHoles>(max_hole_size_->value());
}

void FillHolesModifierWidget::configure(const YAML::Node& config)
{
  max_hole_size_->setValue(getEntry<double>(config, "max_hole_size"));
}

void FillHolesModifierWidget::save(YAML::Node& config) const { config["max_hole_size"] = max_hole_size_->value(); }

}  // namespace noether
