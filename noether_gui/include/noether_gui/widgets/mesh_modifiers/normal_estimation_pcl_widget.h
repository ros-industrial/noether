#pragma once

#include <noether_gui/widgets.h>

namespace Ui
{
class Vector3dEditor;
}

namespace noether
{
class DistanceDoubleSpinBox;

/**
 * @ingroup gui_widgets_mesh_modifiers
 */
class NormalEstimationPCLMeshModifierWidget : public BaseWidget
{
public:
  NormalEstimationPCLMeshModifierWidget(QWidget* parent = nullptr);

  void configure(const YAML::Node& config) override;
  void save(YAML::Node& config) const override;

protected:
  DistanceDoubleSpinBox* radius_;
  Ui::Vector3dEditor* view_point_;
};

}  // namespace noether
