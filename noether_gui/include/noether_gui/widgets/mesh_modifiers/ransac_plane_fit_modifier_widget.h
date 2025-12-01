#pragma once

#include <noether_gui/widgets/mesh_modifiers/ransac_primitive_fit_modifier_widget.h>

namespace noether
{
/**
 * @ingroup gui_widgets_mesh_modifiers
 */
class RansacPlaneProjectionMeshModifierWidget : public RansacPrimitiveFitMeshModifierWidget
{
public:
  using RansacPrimitiveFitMeshModifierWidget::RansacPrimitiveFitMeshModifierWidget;

  void save(YAML::Node&) const override;
};

}  // namespace noether
