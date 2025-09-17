#pragma once

#include <noether_gui/widgets.h>

class QGroupBox;

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
  noether::DistanceDoubleSpinBox* x_dim_spinbox_;
  noether::DistanceDoubleSpinBox* y_dim_spinbox_;
  noether::DistanceDoubleSpinBox* x_spacing_spinbox_;
  noether::DistanceDoubleSpinBox* y_spacing_spinbox_;
};

}  // namespace noether
