#pragma once

#include <noether_gui/widgets.h>
#include <noether_tpp/core/mesh_modifier.h>

namespace noether
{
class NormalsFromMeshFacesMeshModifierWidget : public MeshModifierWidget
{
public:
  using MeshModifierWidget::MeshModifierWidget;

  MeshModifier::ConstPtr create() const override;
};

}  // namespace noether
