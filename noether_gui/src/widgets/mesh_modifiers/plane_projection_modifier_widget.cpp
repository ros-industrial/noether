#include <noether_gui/widgets/mesh_modifiers/plane_projection_modifier_widget.h>
#include <noether_gui/utils.h>

#include <noether_tpp/mesh_modifiers/plane_projection_modifier.h>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QFormLayout>
#include <QLabel>

static std::string DIST_THRESH_KEY = "distance_threshold";
static std::string MAX_PLANES_KEY = "max_planes";
static std::string MIN_VERTICES_KEY = "min_vertices";

namespace noether
{
PlaneProjectionMeshModifierWidget::PlaneProjectionMeshModifierWidget(QWidget* parent)
  : MeshModifierWidget(parent)
  , distance_threshold_(new QDoubleSpinBox(this))
  , max_planes_(new QSpinBox(this))
  , min_vertices_(new QSpinBox(this))
{
  distance_threshold_->setDecimals(3);
  distance_threshold_->setMinimum(0.0);
  distance_threshold_->setSingleStep(0.010);
  distance_threshold_->setValue(0.010);

  max_planes_->setMinimum(-1);
  max_planes_->setMaximum(std::numeric_limits<int>::max());
  max_planes_->setValue(1);

  min_vertices_->setMinimum(0);
  min_vertices_->setMaximum(std::numeric_limits<int>::max());
  min_vertices_->setValue(0);

  auto layout = new QFormLayout(this);
  layout->addRow(new QLabel("Distance threshold (m)", this), distance_threshold_);
  layout->addRow(new QLabel("Maximum number of planes", this), max_planes_);
  layout->addRow(new QLabel("Minimum vertices per plane", this), min_vertices_);
}

MeshModifier::ConstPtr PlaneProjectionMeshModifierWidget::create() const
{
  return std::make_unique<PlaneProjectionMeshModifier>(
      distance_threshold_->value(), max_planes_->value(), min_vertices_->value());
}

void PlaneProjectionMeshModifierWidget::configure(const YAML::Node& node)
{
  distance_threshold_->setValue(getEntry<double>(node, DIST_THRESH_KEY));
  max_planes_->setValue(getEntry<int>(node, MAX_PLANES_KEY));
  min_vertices_->setValue(getEntry<int>(node, MIN_VERTICES_KEY));
}

void PlaneProjectionMeshModifierWidget::save(YAML::Node& node) const
{
  node[DIST_THRESH_KEY] = distance_threshold_->value();
  node[MAX_PLANES_KEY] = max_planes_->value();
  node[MIN_VERTICES_KEY] = min_vertices_->value();
}

}  // namespace noether
