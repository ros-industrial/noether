#pragma once

#include <noether_gui/widgets/tool_path_planners/raster/plane_slicer_raster_planner_widget.h>

namespace noether
{
class AngleDoubleSpinBox;

/**
 * @ingroup gui_widgets_tool_path_planners
 */
class CrossHatchPlaneSlicerRasterPlannerWidget : public PlaneSlicerRasterPlannerWidget
{
public:
  CrossHatchPlaneSlicerRasterPlannerWidget(boost_plugin_loader::PluginLoader&& loader, QWidget* parent = nullptr);

  void configure(const YAML::Node&) override;
  void save(YAML::Node&) const override;

private:
  AngleDoubleSpinBox* cross_hatch_angle_;
};

}  // namespace noether
