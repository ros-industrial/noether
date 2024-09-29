#include <noether_gui/widgets/mesh_modifiers/normals_from_mesh_faces_modifier_widget.h>

#include <noether_tpp/mesh_modifiers/normals_from_mesh_faces_modifier.h>

namespace noether
{
MeshModifier::ConstPtr NormalsFromMeshFacesMeshModifierWidget::create() const
{
  return std::make_unique<NormalsFromMeshFacesMeshModifier>();
}

}  // namespace noether
