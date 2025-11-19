#include <noether_gui/widgets/mesh_modifiers/ransac_plane_fit_modifier_widget.h>

namespace noether
{
void RansacPlaneProjectionMeshModifierWidget::save(YAML::Node& node) const
{
  node["name"] = "RansacPlaneProjection";
  RansacPrimitiveFitMeshModifierWidget::save(node);
}

}  // namespace noether
