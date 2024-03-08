#include <noether_gui/widgets/mesh_modifiers/plane_projection_modifier_widget.h>
#include <noether_gui/utils.h>

#include <noether_tpp/mesh_modifiers/plane_projection_modifier.h>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QLabel>

static std::string DIST_THRESH_KEY = "distance_threshold";

namespace noether
{
PlaneProjectionMeshModifierWidget::PlaneProjectionMeshModifierWidget(QWidget* parent)
  : MeshModifierWidget(parent), distance_threshold_(new QDoubleSpinBox(this))
{
  distance_threshold_->setDecimals(3);
  distance_threshold_->setMinimum(0.0);
  distance_threshold_->setSingleStep(0.010);
  distance_threshold_->setValue(0.010);

  auto layout = new QFormLayout(this);
  layout->addRow(new QLabel("Distance threshold (m)", this), distance_threshold_);
}

MeshModifier::ConstPtr PlaneProjectionMeshModifierWidget::create() const
{
  return std::make_unique<PlaneProjectionMeshModifier>(distance_threshold_->value());
}

void PlaneProjectionMeshModifierWidget::configure(const YAML::Node& node)
{
  distance_threshold_->setValue(getEntry<double>(node, DIST_THRESH_KEY));
}

void PlaneProjectionMeshModifierWidget::save(YAML::Node& node) const
{
  node[DIST_THRESH_KEY] = distance_threshold_->value();
}

}  // namespace noether
