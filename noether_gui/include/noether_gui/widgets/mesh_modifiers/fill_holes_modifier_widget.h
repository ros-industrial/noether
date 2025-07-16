#pragma once

#include <noether_gui/widgets.h>

namespace noether
{
class DistanceDoubleSpinBox;

/**
 * @ingroup gui_widgets_mesh_modifiers
 */
class FillHolesModifierWidget : public MeshModifierWidget
{
public:
  FillHolesModifierWidget(QWidget* parent = nullptr);

  void configure(const YAML::Node& config) override;
  void save(YAML::Node& config) const override;

protected:
  DistanceDoubleSpinBox* max_hole_size_;
};

}  // namespace noether
