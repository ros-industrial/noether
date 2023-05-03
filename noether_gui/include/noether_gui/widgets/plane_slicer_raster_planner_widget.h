#pragma once

#include <noether_gui/widgets/raster_planner_widget.h>

class QDoubleSpinBox;

namespace noether
{
class PlaneSlicerRasterPlannerWidget : public RasterPlannerWidget
{
  Q_OBJECT
public:
  PlaneSlicerRasterPlannerWidget(boost_plugin_loader::PluginLoader&& loader,
                                 QWidget* parent = nullptr,
                                 const double line_spacing = 0.1,
                                 const double point_spacing = 0.025,
                                 const double min_hole_size = 0.1,
                                 const double search_radius = 0.025,
                                 const double min_segment_size = 0.1);

  ToolPathPlanner::ConstPtr create() const override final;

private:
  QDoubleSpinBox* search_radius_;
  QDoubleSpinBox* min_segment_size_;
};

}  // namespace noether
