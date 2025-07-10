#include <noether_gui/widgets/mesh_modifiers/plane_projection_modifier_widget.h>
#include <noether_gui/widgets/distance_double_spin_box.h>
#include <noether_gui/utils.h>

#include <noether_tpp/mesh_modifiers/plane_projection_modifier.h>
#include <QSpinBox>
#include <QFormLayout>
#include <QLabel>

static const std::string DIST_THRESH_KEY = "distance_threshold";
static const std::string MAX_PLANES_KEY = "max_planes";
static const std::string MIN_VERTICES_KEY = "min_vertices";

namespace noether
{
PlaneProjectionMeshModifierWidget::PlaneProjectionMeshModifierWidget(QWidget* parent)
  : MeshModifierWidget(parent)
  , distance_threshold_(new DistanceDoubleSpinBox(this))
  , max_planes_(new QSpinBox(this))
  , min_vertices_(new QSpinBox(this))
{
  distance_threshold_->setDecimals(3);
  distance_threshold_->setMinimum(0.0);
  distance_threshold_->setSingleStep(0.010);
  distance_threshold_->setValue(0.010);

  max_planes_->setMinimum(0);
  max_planes_->setMaximum(std::numeric_limits<int>::max());
  max_planes_->setValue(1);

  min_vertices_->setMinimum(1);
  min_vertices_->setMaximum(std::numeric_limits<int>::max());
  min_vertices_->setValue(1);

  auto layout = new QFormLayout(this);
  layout->addRow(new QLabel("Distance threshold", this), distance_threshold_);
  layout->addRow(new QLabel("Maximum number of planes", this), max_planes_);
  layout->addRow(new QLabel("Minimum vertices per plane", this), min_vertices_);
}

MeshModifier::ConstPtr PlaneProjectionMeshModifierWidget::create() const
{
  return std::make_unique<PlaneProjectionMeshModifier>(distance_threshold_->value(),
                                                       static_cast<unsigned>(max_planes_->value()),
                                                       static_cast<unsigned>(min_vertices_->value()));
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
