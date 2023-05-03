#pragma once

#include <noether_gui/widgets/raster_planner_widget.h>

class QDoubleSpinBox;

namespace noether
{
class PlaneSlicerRasterPlannerWidget : public RasterPlannerWidget
{
  Q_OBJECT
public:
  PlaneSlicerRasterPlannerWidget(boost_plugin_loader::PluginLoader&& loader, QWidget* parent = nullptr);

  ToolPathPlanner::ConstPtr create() const override final;

  void configure(const YAML::Node&) override;

private:
  QDoubleSpinBox* search_radius_;
  QDoubleSpinBox* min_segment_size_;
};

}  // namespace noether
