#pragma once

#include <noether_gui/widgets.h>

namespace noether
{
/**
 * @ingroup gui_widgets_mesh_modifiers
 */
class NormalsFromMeshFacesMeshModifierWidget : public MeshModifierWidget
{
public:
  using MeshModifierWidget::MeshModifierWidget;

  void save(YAML::Node&) const override;
};

}  // namespace noether
