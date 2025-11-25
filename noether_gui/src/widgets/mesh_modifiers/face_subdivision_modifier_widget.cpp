#include <noether_gui/widgets/mesh_modifiers/face_subdivision_modifier_widget.h>
#include <noether_gui/widgets/distance_double_spin_box.h>

#include <noether_tpp/serialization.h>
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

  auto label = new QLabel("Iterations", this);
  label->setToolTip("Number of subdivision iterations to run");

  form_layout->addRow(label, n_iterations_);
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
  : BaseWidget(parent), triangle_characteristic_length_(new DistanceDoubleSpinBox(this))
{
  auto* form_layout = new QFormLayout(this);
  triangle_characteristic_length_->setMinimum(0.0);
  triangle_characteristic_length_->setValue(0.01);
  triangle_characteristic_length_->setSingleStep(0.005);

  auto label = new QLabel("Triangle Characteristic Length", this);
  label->setToolTip("Characteristic length of a triangle (l), such that the area (A) of the triangle is: A = 0.5 * "
                    "l^2. This roughly equates to the maximum allowable triangle edge length (assuming an equilateral "
                    "triangle).");

  form_layout->addRow(label, triangle_characteristic_length_);
}

void FaceSubdivisionByAreaMeshModifierWidget::configure(const YAML::Node& config)
{
  const auto max_area = YAML::getMember<double>(config, "max_area");
  triangle_characteristic_length_->setValue(std::sqrt(2 * max_area));
}

void FaceSubdivisionByAreaMeshModifierWidget::save(YAML::Node& config) const
{
  config["name"] = "FaceSubdivisionByArea";
  config["max_area"] = 0.5 * std::pow(triangle_characteristic_length_->value(), 2.0);
}

}  // namespace noether
