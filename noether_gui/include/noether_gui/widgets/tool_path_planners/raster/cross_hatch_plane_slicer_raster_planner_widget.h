#pragma once

#include <noether_gui/widgets/tool_path_planners/raster/plane_slicer_raster_planner_widget.h>

class QDoubleSpinBox;

namespace noether
{
class CrossHatchPlaneSlicerRasterPlannerWidget : public PlaneSlicerRasterPlannerWidget
{
public:
  CrossHatchPlaneSlicerRasterPlannerWidget(boost_plugin_loader::PluginLoader&& loader, QWidget* parent = nullptr);

  ToolPathPlanner::ConstPtr create() const override;

  void configure(const YAML::Node&) override;
  void save(YAML::Node&) const override;

private:
  QDoubleSpinBox* cross_hatch_angle_;
};

}  // namespace noether
