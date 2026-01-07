#pragma once

#include <noether_gui/widgets/tool_path_planners/raster/raster_planner_widget.h>

class QCheckBox;

namespace Ui
{
class PlaneSlicerRasterPlanner;
}

namespace noether
{
class DistanceDoubleSpinBox;

/**
 * @ingroup gui_widgets_tool_path_planners
 */
class PlaneSlicerRasterPlannerWidget : public RasterPlannerWidget
{
public:
  PlaneSlicerRasterPlannerWidget(std::shared_ptr<const WidgetFactory> factory, QWidget* parent = nullptr);

  void configure(const YAML::Node&) override;
  void save(YAML::Node&) const override;

protected:
  Ui::PlaneSlicerRasterPlanner* ui_plane_slicer_;
};

}  // namespace noether
