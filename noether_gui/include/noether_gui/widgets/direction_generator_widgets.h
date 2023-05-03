#pragma once

#include <noether_gui/widgets.h>

#include <noether_tpp/tool_path_planners/raster/raster_planner.h>

class QDoubleSpinBox;
class QFormLayout;
class QLabel;

namespace Ui
{
class Vector3dEditor;
}

namespace noether
{
class FixedDirectionGeneratorWidget : public DirectionGeneratorWidget
{
  Q_OBJECT
public:
  FixedDirectionGeneratorWidget(QWidget* parent = nullptr, const Eigen::Vector3d& = Eigen::Vector3d::UnitX());

  DirectionGenerator::ConstPtr create() const override;

private:
  Ui::Vector3dEditor* ui_;
};

class PrincipalAxisDirectionGeneratorWidget : public DirectionGeneratorWidget
{
  Q_OBJECT
public:
  PrincipalAxisDirectionGeneratorWidget(QWidget* parent = nullptr, const double rotation_offset = 0.0);

  DirectionGenerator::ConstPtr create() const override;

private:
  QFormLayout* layout_;
  QLabel* label_;
  QDoubleSpinBox* rotation_offset_;
};

}  // namespace noether
