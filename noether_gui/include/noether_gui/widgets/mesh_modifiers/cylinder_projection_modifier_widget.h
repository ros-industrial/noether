#pragma once

#include <noether_gui/widgets.h>

namespace Ui
{
class CylinderSegmentation;
class Vector3dEditor;
}  // namespace Ui

namespace noether
{
class CylinderProjectionModifierWidget : public BaseWidget
{
public:
  CylinderProjectionModifierWidget(QWidget* parent = nullptr);

  void configure(const YAML::Node& config) override;
  void save(YAML::Node& config) const override;

protected:
  Ui::CylinderSegmentation* ui_;
  Ui::Vector3dEditor* axis_;
};

}  // namespace noether
