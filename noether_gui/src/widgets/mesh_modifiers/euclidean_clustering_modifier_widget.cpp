#include <noether_gui/widgets/mesh_modifiers/euclidean_clustering_modifier_widget.h>
#include <noether_gui/utils.h>

#include <noether_tpp/mesh_modifiers/euclidean_clustering_modifier.h>
#include <QDoubleSpinBox>
#include <QSpinBox>
#include <QFormLayout>
#include <QLabel>

static std::string DIST_THRESH_KEY = "distance_threshold";
static std::string MIN_SIZE_KEY = "min_cluster_size";
static std::string MAX_SIZE_KEY = "max_cluster_size";

namespace noether
{
EuclideanClusteringMeshModifierWidget::EuclideanClusteringMeshModifierWidget(QWidget* parent)
  : MeshModifierWidget(parent)
  , tolerance_(new QDoubleSpinBox(this))
  , min_cluster_size_(new QSpinBox(this))
  , max_cluster_size_(new QSpinBox(this))
{
  tolerance_->setDecimals(3);
  tolerance_->setMinimum(0.0);
  tolerance_->setSingleStep(0.010);
  tolerance_->setValue(0.010);

  min_cluster_size_->setMinimum(0);
  min_cluster_size_->setMaximum(std::numeric_limits<int>::max());
  min_cluster_size_->setValue(0);

  max_cluster_size_->setMinimum(-1);
  max_cluster_size_->setMaximum(std::numeric_limits<int>::max());
  max_cluster_size_->setValue(-1);

  auto layout = new QFormLayout(this);
  layout->addRow(new QLabel("Cluster tolerance (m)", this), tolerance_);
  layout->addRow(new QLabel("Minimum cluster size", this), min_cluster_size_);
  layout->addRow(new QLabel("Maximum cluster size", this), max_cluster_size_);
}

MeshModifier::ConstPtr EuclideanClusteringMeshModifierWidget::create() const
{
  return std::make_unique<EuclideanClusteringMeshModifier>(
      tolerance_->value(), min_cluster_size_->value(), max_cluster_size_->value());
}

void EuclideanClusteringMeshModifierWidget::configure(const YAML::Node& node)
{
  tolerance_->setValue(getEntry<double>(node, DIST_THRESH_KEY));
  min_cluster_size_->setValue(getEntry<int>(node, MIN_SIZE_KEY));
  max_cluster_size_->setValue(getEntry<int>(node, MAX_SIZE_KEY));
}

void EuclideanClusteringMeshModifierWidget::save(YAML::Node& node) const
{
  node[DIST_THRESH_KEY] = tolerance_->value();
  node[MIN_SIZE_KEY] = min_cluster_size_->value();
  node[MAX_SIZE_KEY] = max_cluster_size_->value();
}

}  // namespace noether
