#pragma once

#include <noether_gui/widgets.h>

#include <noether_tpp/tool_path_planners/raster/raster_planner.h>

class QDoubleSpinBox;
class QFormLayout;
class QLabel;

namespace noether
{
class PrincipalAxisDirectionGeneratorWidget : public DirectionGeneratorWidget
{
  Q_OBJECT
public:
  PrincipalAxisDirectionGeneratorWidget(QWidget* parent = nullptr);

  DirectionGenerator::ConstPtr create() const override;

  void configure(const YAML::Node&) override;
  void save(YAML::Node&) const override;

private:
  QFormLayout* layout_;
  QLabel* label_;
  QDoubleSpinBox* rotation_offset_;
};

}  // namespace noether
