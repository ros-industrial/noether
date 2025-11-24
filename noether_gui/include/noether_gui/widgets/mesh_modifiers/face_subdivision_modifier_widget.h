#pragma once

#include <noether_gui/widgets.h>

class QDoubleSpinBox;
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
  QDoubleSpinBox* max_area_;
};

class FaceSubdivisionByEdgeLengthMeshModifierWidget : public BaseWidget
{
public:
  FaceSubdivisionByEdgeLengthMeshModifierWidget(QWidget* parent = nullptr);

  void configure(const YAML::Node& config) override;
  void save(YAML::Node& config) const override;

protected:
  DistanceDoubleSpinBox* max_edge_length_;
  DistanceDoubleSpinBox* min_edge_length_;
};

}  // namespace noether
