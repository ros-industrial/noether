#pragma once

#include <noether_gui/widgets.h>

class QDoubleSpinBox;

namespace noether
{
class FaceSubdivisionMeshModifierWidget : public BaseWidget
{
public:
  FaceSubdivisionMeshModifierWidget(QWidget* parent = nullptr);

  void configure(const YAML::Node& config) override;
  void save(YAML::Node& config) const override;

protected:
  QDoubleSpinBox* max_area_;
};

}  // namespace noether
