#pragma once

#include <noether_gui/widgets/tool_path_planners/raster/raster_planner_widget.h>

class QCheckBox;

namespace Ui
{
class PlaneSlicerLegacyRasterPlanner;
}

namespace noether
{
/**
 * @ingroup gui_widgets_tool_path_planners
 */
class PlaneSlicerLegacyRasterPlannerWidget : public RasterPlannerWidget
{
public:
  PlaneSlicerLegacyRasterPlannerWidget(std::shared_ptr<const WidgetFactory> factory, QWidget* parent = nullptr);

  void configure(const YAML::Node&) override;
  void save(YAML::Node&) const override;

protected:
  Ui::PlaneSlicerLegacyRasterPlanner* ui_plane_slicer_;
};

}  // namespace noether
