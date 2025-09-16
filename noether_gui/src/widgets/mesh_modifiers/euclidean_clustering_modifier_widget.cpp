#include <noether_gui/widgets/mesh_modifiers/euclidean_clustering_modifier_widget.h>
#include <noether_gui/widgets/distance_double_spin_box.h>

#include <noether_tpp/serialization.h>
#include <QSpinBox>
#include <QFormLayout>
#include <QLabel>

static const std::string TOLERANCE_KEY = "tolerance";
static const std::string MIN_SIZE_KEY = "min_cluster_size";
static const std::string MAX_SIZE_KEY = "max_cluster_size";

namespace noether
{
EuclideanClusteringMeshModifierWidget::EuclideanClusteringMeshModifierWidget(QWidget* parent)
  : BaseWidget(parent)
  , tolerance_(new DistanceDoubleSpinBox(this))
  , min_cluster_size_(new QSpinBox(this))
  , max_cluster_size_(new QSpinBox(this))
{
  tolerance_->setDecimals(3);
  tolerance_->setMinimum(0.0);
  tolerance_->setSingleStep(0.010);
  tolerance_->setValue(0.010);

  min_cluster_size_->setMinimum(1);
  min_cluster_size_->setMaximum(std::numeric_limits<int>::max());
  min_cluster_size_->setValue(1);

  max_cluster_size_->setMinimum(-1);
  max_cluster_size_->setMaximum(std::numeric_limits<int>::max());
  max_cluster_size_->setValue(-1);

  auto layout = new QFormLayout(this);
  layout->addRow(new QLabel("Cluster tolerance", this), tolerance_);
  layout->addRow(new QLabel("Minimum cluster size", this), min_cluster_size_);
  layout->addRow(new QLabel("Maximum cluster size", this), max_cluster_size_);
}

void EuclideanClusteringMeshModifierWidget::configure(const YAML::Node& node)
{
  tolerance_->setValue(YAML::getMember<double>(node, TOLERANCE_KEY));
  min_cluster_size_->setValue(YAML::getMember<int>(node, MIN_SIZE_KEY));
  max_cluster_size_->setValue(YAML::getMember<int>(node, MAX_SIZE_KEY));
}

void EuclideanClusteringMeshModifierWidget::save(YAML::Node& node) const
{
  node["name"] = "EuclideanClustering";
  node[TOLERANCE_KEY] = tolerance_->value();
  node[MIN_SIZE_KEY] = min_cluster_size_->value();
  node[MAX_SIZE_KEY] = max_cluster_size_->value();
}

}  // namespace noether
