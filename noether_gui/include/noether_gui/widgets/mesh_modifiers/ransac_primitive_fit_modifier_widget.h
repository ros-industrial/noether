#pragma once

#include <noether_gui/widgets.h>

namespace Ui
{
class RansacPrimitiveFit;
class Vector3dEditor;
}  // namespace Ui

namespace noether
{
class RansacPrimitiveFitMeshModifierWidget : public BaseWidget
{
public:
  RansacPrimitiveFitMeshModifierWidget(QWidget* parent = nullptr);

  void configure(const YAML::Node& config) override;
  void save(YAML::Node& config) const override;

protected:
  Ui::RansacPrimitiveFit* ui_;
  Ui::Vector3dEditor* axis_;
};

}  // namespace noether
