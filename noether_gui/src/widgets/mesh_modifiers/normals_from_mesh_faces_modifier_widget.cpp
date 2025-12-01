#include <noether_gui/widgets/mesh_modifiers/normals_from_mesh_faces_modifier_widget.h>

namespace noether
{
void NormalsFromMeshFacesMeshModifierWidget::save(YAML::Node& config) const { config["name"] = "NormalsFromMeshFaces"; }

}  // namespace noether
