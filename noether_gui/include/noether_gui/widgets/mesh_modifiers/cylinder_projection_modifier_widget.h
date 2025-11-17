#pragma once

#include <noether_gui/widgets/mesh_modifiers/ransac_primitive_fit_modifier_widget.h>

namespace Ui
{
class CylinderProjection;
class Vector3dEditor;
}  // namespace Ui

namespace noether
{
class CylinderProjectionModifierWidget : public RansacPrimitiveFitModifierWidget
{
public:
  CylinderProjectionModifierWidget(QWidget* parent = nullptr);

  void configure(const YAML::Node& config) override;
  void save(YAML::Node& config) const override;

protected:
  Ui::CylinderProjection* model_ui_;
  Ui::Vector3dEditor* axis_;
};

}  // namespace noether
