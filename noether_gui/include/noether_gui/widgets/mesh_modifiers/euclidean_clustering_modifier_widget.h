#pragma once

#include <noether_gui/widgets.h>

class QSpinBox;

namespace noether
{
class DistanceDoubleSpinBox;

/**
 * @ingroup gui_widgets_mesh_modifiers
 */
class EuclideanClusteringMeshModifierWidget : public MeshModifierWidget
{
public:
  EuclideanClusteringMeshModifierWidget(QWidget* parent = nullptr);

  void configure(const YAML::Node&) override;
  void save(YAML::Node&) const override;

private:
  DistanceDoubleSpinBox* tolerance_;
  QSpinBox* min_cluster_size_;
  QSpinBox* max_cluster_size_;
};

}  // namespace noether
