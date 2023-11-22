#pragma once

#include <noether_gui/widgets.h>
#include <noether_tpp/core/tool_path_planner.h>

class QGroupBox;

namespace Ui
{
class Vector3dEditor;
class QuaternionEditor;
}  // namespace Ui

namespace noether
{
class DistanceDoubleSpinBox;

class FlatPlaneToolPathPlannerWidget : public BaseWidget
{
public:
  FlatPlaneToolPathPlannerWidget(QWidget* parent = nullptr);

  void configure(const YAML::Node&) override;
  void save(YAML::Node&) const override;

private:
  noether::DistanceDoubleSpinBox* plane_x_dim_spinbox_;
  noether::DistanceDoubleSpinBox* plane_y_dim_spinbox_;
  noether::DistanceDoubleSpinBox* plane_x_spacing_spinbox_;
  noether::DistanceDoubleSpinBox* plane_y_spacing_spinbox_;
};

}  // namespace noether
