#pragma once

#include <noether_gui/widgets.h>

class QSpinBox;

namespace noether
{
class DistanceDoubleSpinBox;

class FaceMidpointSubdivisionMeshModifierWidget : public BaseWidget
{
public:
  FaceMidpointSubdivisionMeshModifierWidget(QWidget* parent = nullptr);

  void configure(const YAML::Node& config) override;
  void save(YAML::Node& config) const override;

protected:
  QSpinBox* n_iterations_;
};

class FaceSubdivisionByAreaMeshModifierWidget : public BaseWidget
{
public:
  FaceSubdivisionByAreaMeshModifierWidget(QWidget* parent = nullptr);

  void configure(const YAML::Node& config) override;
  void save(YAML::Node& config) const override;

protected:
  DistanceDoubleSpinBox* triangle_characteristic_length_;
};

}  // namespace noether
