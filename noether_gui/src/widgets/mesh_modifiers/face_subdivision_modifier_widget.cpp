#include <noether_gui/widgets/mesh_modifiers/face_subdivision_modifier_widget.h>
#include <noether_gui/widgets/distance_double_spin_box.h>

#include <noether_tpp/serialization.h>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>

namespace noether
{
FaceMidpointSubdivisionMeshModifierWidget::FaceMidpointSubdivisionMeshModifierWidget(QWidget* parent)
  : BaseWidget(parent), n_iterations_(new QSpinBox(this))
{
  auto* form_layout = new QFormLayout(this);
  n_iterations_->setRange(1, 10);
  n_iterations_->setValue(1);
  n_iterations_->setSingleStep(1);
  n_iterations_->setToolTip("Number of subdivision iterations to run");
  form_layout->addRow(new QLabel("Iterations", this), n_iterations_);
}

void FaceMidpointSubdivisionMeshModifierWidget::configure(const YAML::Node& config)
{
  n_iterations_->setValue(YAML::getMember<double>(config, "n_iterations"));
}

void FaceMidpointSubdivisionMeshModifierWidget::save(YAML::Node& config) const
{
  config["name"] = "FaceMidpointSubdivision";
  config["n_iterations"] = n_iterations_->value();
}

FaceSubdivisionByAreaMeshModifierWidget::FaceSubdivisionByAreaMeshModifierWidget(QWidget* parent)
  : BaseWidget(parent), max_area_(new QDoubleSpinBox(this))
{
  auto* form_layout = new QFormLayout(this);
  max_area_->setMinimum(0.0);
  max_area_->setValue(0.01);
  max_area_->setSingleStep(0.005);
  max_area_->setDecimals(6);
  max_area_->setSuffix(" m^2");
  form_layout->addRow(new QLabel("Max Area", this), max_area_);
}

void FaceSubdivisionByAreaMeshModifierWidget::configure(const YAML::Node& config)
{
  max_area_->setValue(YAML::getMember<double>(config, "max_area"));
}

void FaceSubdivisionByAreaMeshModifierWidget::save(YAML::Node& config) const
{
  config["name"] = "FaceSubdivisionByArea";
  config["max_area"] = max_area_->value();
}

}  // namespace noether
