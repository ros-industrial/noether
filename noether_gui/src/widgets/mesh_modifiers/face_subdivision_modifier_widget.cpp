#include <noether_gui/widgets/mesh_modifiers/face_subdivision_modifier_widget.h>
#include <noether_tpp/serialization.h>

#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>

namespace noether
{
FaceSubdivisionMeshModifierWidget::FaceSubdivisionMeshModifierWidget(QWidget* parent)
  : BaseWidget(parent), max_area_(new QDoubleSpinBox(this))
{
  auto* layout = new QVBoxLayout(this);

  auto* form_layout = new QFormLayout(this);
  max_area_->setMinimum(0.0);
  max_area_->setValue(0.01);
  max_area_->setSingleStep(0.005);
  max_area_->setDecimals(4);
  max_area_->setSuffix(" m^2");
  form_layout->addRow(new QLabel("Max Area", this), max_area_);
  layout->addLayout(form_layout);
}

void FaceSubdivisionMeshModifierWidget::configure(const YAML::Node& config)
{
  max_area_->setValue(YAML::getMember<double>(config, "max_area"));
}

void FaceSubdivisionMeshModifierWidget::save(YAML::Node& config) const
{
  config["name"] = "FaceSubdivision";
  config["max_area"] = max_area_->value();
}

}  // namespace noether
