#include <noether_gui/widgets/mesh_modifiers/normal_estimation_pcl_widget.h>
#include "../ui_vector3d_editor_widget.h"
#include <noether_gui/utils.h>

#include <noether_tpp/mesh_modifiers/normal_estimation_pcl.h>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QVBoxLayout>
#include <QLabel>

namespace noether
{
NormalEstimationPCLMeshModifierWidget::NormalEstimationPCLMeshModifierWidget(QWidget* parent)
  : MeshModifierWidget(parent), radius_(new QDoubleSpinBox(this)), view_point_(new Ui::Vector3dEditor())
{
  auto* layout = new QVBoxLayout(this);

  // Create a form layout with the normals radius parameter
  {
    auto* form_layout = new QFormLayout();

    radius_->setMinimum(0.0);
    radius_->setValue(0.010);
    radius_->setSingleStep(0.010);
    radius_->setDecimals(3);

    form_layout->addRow("Radius (m)", radius_);
    layout->addLayout(form_layout);
  }

  // Set up the Vector3d editor
  auto* widget = new QWidget(this);
  view_point_->setupUi(widget);
  view_point_->group_box->setTitle("View Point");

  layout->addWidget(widget);
}

MeshModifier::ConstPtr NormalEstimationPCLMeshModifierWidget::create() const
{
  return std::make_unique<NormalEstimationPCLMeshModifier>(radius_->value(),
                                                           view_point_->double_spin_box_x->value(),
                                                           view_point_->double_spin_box_y->value(),
                                                           view_point_->double_spin_box_z->value());
}

void NormalEstimationPCLMeshModifierWidget::configure(const YAML::Node& config)
{
  radius_->setValue(getEntry<double>(config, "radius"));
  view_point_->double_spin_box_x->setValue(getEntry<double>(config, "vx"));
  view_point_->double_spin_box_y->setValue(getEntry<double>(config, "vy"));
  view_point_->double_spin_box_z->setValue(getEntry<double>(config, "vz"));
}

void NormalEstimationPCLMeshModifierWidget::save(YAML::Node& config) const
{
  config["radius"] = radius_->value();
  config["vx"] = view_point_->double_spin_box_x->value();
  config["vy"] = view_point_->double_spin_box_y->value();
  config["vz"] = view_point_->double_spin_box_z->value();
}

}  // namespace noether
