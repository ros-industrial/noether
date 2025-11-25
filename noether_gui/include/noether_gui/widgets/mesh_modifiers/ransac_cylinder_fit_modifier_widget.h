#pragma once

#include <noether_gui/widgets/mesh_modifiers/ransac_primitive_fit_modifier_widget.h>

class QSpinBox;
class QCheckBox;

namespace Ui
{
class RansacCylinderProjection;
class Vector3dEditor;
}  // namespace Ui

namespace noether
{
class RansacCylinderProjectionMeshModifierWidget : public RansacPrimitiveFitMeshModifierWidget
{
public:
  RansacCylinderProjectionMeshModifierWidget(QWidget* parent = nullptr);

  void configure(const YAML::Node& config) override;
  void save(YAML::Node& config) const override;

protected:
  Ui::RansacCylinderProjection* model_ui_;
  Ui::Vector3dEditor* axis_;
};

class RansacCylinderFitMeshModifierWidget : public RansacCylinderProjectionMeshModifierWidget
{
public:
  RansacCylinderFitMeshModifierWidget(QWidget* parent);

  void configure(const YAML::Node& config) override;
  void save(YAML::Node& config) const override;

protected:
  QSpinBox* resolution_;
  QCheckBox* include_caps_;
  QCheckBox* uniform_triangles_;
};

}  // namespace noether
