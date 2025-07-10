#pragma once

#include <noether_gui/widgets/tool_path_planners/raster/raster_planner_widget.h>

class QCheckBox;

namespace noether
{
class DistanceDoubleSpinBox;

class PlaneSlicerRasterPlannerWidget : public RasterPlannerWidget
{
public:
  PlaneSlicerRasterPlannerWidget(boost_plugin_loader::PluginLoader&& loader, QWidget* parent = nullptr);

  ToolPathPlanner::ConstPtr create() const override;

  void configure(const YAML::Node&) override;
  void save(YAML::Node&) const override;

protected:
  DistanceDoubleSpinBox* search_radius_;
  DistanceDoubleSpinBox* min_segment_size_;
  QCheckBox* bidirectional_;
};

}  // namespace noether
