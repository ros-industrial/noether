#pragma once

#include <noether_gui/widgets.h>
#include <noether_tpp/core/mesh_modifier.h>

class QDoubleSpinBox;
class QSpinBox;

namespace noether
{
class PlaneProjectionMeshModifierWidget : public MeshModifierWidget
{
  Q_OBJECT
public:
  PlaneProjectionMeshModifierWidget(QWidget* parent = nullptr);

  MeshModifier::ConstPtr create() const override;

  void configure(const YAML::Node&) override;
  void save(YAML::Node&) const override;

private:
  QDoubleSpinBox* distance_threshold_;
  QSpinBox* max_planes_;
  QSpinBox* min_vertices_;
};

}  // namespace noether
