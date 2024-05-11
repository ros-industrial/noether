#pragma once

#include <noether_gui/widgets.h>
#include <noether_tpp/core/mesh_modifier.h>

class QDoubleSpinBox;
class QSpinBox;

namespace noether
{
class EuclideanClusteringMeshModifierWidget : public MeshModifierWidget
{
public:
  EuclideanClusteringMeshModifierWidget(QWidget* parent = nullptr);

  MeshModifier::ConstPtr create() const override;

  void configure(const YAML::Node&) override;
  void save(YAML::Node&) const override;

private:
  QDoubleSpinBox* tolerance_;
  QSpinBox* min_cluster_size_;
  QSpinBox* max_cluster_size_;
};

}  // namespace noether
