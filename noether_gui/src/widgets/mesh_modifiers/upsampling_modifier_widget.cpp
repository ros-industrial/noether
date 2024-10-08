#include <noether_gui/widgets/mesh_modifiers/upsampling_modifier_widget.h>
#include <noether_tpp/mesh_modifiers/upsampling_modifier.h>
#include <noether_gui/utils.h>

#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>

namespace noether
{
UpsamplingMeshModifierWidget::UpsamplingMeshModifierWidget(QWidget* parent)
  : MeshModifierWidget(parent), max_area_(new QDoubleSpinBox(this))
{
  auto* layout = new QVBoxLayout(this);

  auto* form_layout = new QFormLayout(this);
  max_area_->setMinimum(0.0);
  max_area_->setValue(0.01);
  max_area_->setSingleStep(0.005);
  max_area_->setDecimals(4);
  form_layout->addRow(new QLabel("Max Area (m^2)", this), max_area_);
  layout->addLayout(form_layout);
}

MeshModifier::ConstPtr UpsamplingMeshModifierWidget::create() const
{
  return std::make_unique<UpsamplingMeshModifier>(max_area_->value());
}

void UpsamplingMeshModifierWidget::configure(const YAML::Node& config)
{
  max_area_->setValue(getEntry<double>(config, "max_area"));
}

void UpsamplingMeshModifierWidget::save(YAML::Node& config) const { config["max_area"] = max_area_->value(); }

}  // namespace noether
