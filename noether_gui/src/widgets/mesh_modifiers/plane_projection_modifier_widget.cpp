#include <noether_gui/widgets/mesh_modifiers/plane_projection_modifier_widget.h>

namespace noether
{
void PlaneProjectionMeshModifierWidget::save(YAML::Node& node) const
{
  node["name"] = "PlaneProjection";
  RansacPrimitiveFitModifierWidget::save(node);
}

}  // namespace noether
