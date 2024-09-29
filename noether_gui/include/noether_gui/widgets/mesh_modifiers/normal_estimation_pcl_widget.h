#pragma once

#include <noether_gui/widgets.h>
#include <noether_tpp/core/mesh_modifier.h>

class QDoubleSpinBox;

namespace Ui
{
class Vector3dEditor;
}

namespace noether
{
class NormalEstimationPCLMeshModifierWidget : public MeshModifierWidget
{
public:
  NormalEstimationPCLMeshModifierWidget(QWidget* parent = nullptr);

  MeshModifier::ConstPtr create() const override;
  void configure(const YAML::Node& config) override;
  void save(YAML::Node& config) const override;

protected:
  QDoubleSpinBox* radius_;
  Ui::Vector3dEditor* view_point_;
};

}  // namespace noether
