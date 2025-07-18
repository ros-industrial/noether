#pragma once

#include <noether_gui/widgets.h>

class QSpinBox;

namespace noether
{
class DistanceDoubleSpinBox;

/**
 * @ingroup gui_widgets_mesh_modifiers
 */
class PlaneProjectionMeshModifierWidget : public BaseWidget
{
public:
  PlaneProjectionMeshModifierWidget(QWidget* parent = nullptr);

  void configure(const YAML::Node&) override;
  void save(YAML::Node&) const override;

private:
  DistanceDoubleSpinBox* distance_threshold_;
  QSpinBox* max_planes_;
  QSpinBox* min_vertices_;
};

}  // namespace noether
