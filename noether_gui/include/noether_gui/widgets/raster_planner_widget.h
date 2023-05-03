#pragma once

#include <noether_gui/widgets.h>

#include <noether_tpp/tool_path_planners/raster/raster_planner.h>
#include <boost_plugin_loader/plugin_loader.h>

namespace Ui
{
class RasterPlanner;
}

namespace noether
{
class RasterPlannerWidget : public ToolPathPlannerWidget
{
public:
  RasterPlannerWidget(boost_plugin_loader::PluginLoader&& loader,
                      QWidget* parent = nullptr,
                      const double line_spacing = 0.1,
                      const double point_spacing = 0.025,
                      const double min_hole_size = 0.1);

protected:
  DirectionGeneratorWidget* getDirectionGeneratorWidget() const;
  OriginGeneratorWidget* getOriginGeneratorWidget() const;

  const boost_plugin_loader::PluginLoader loader_;
  Ui::RasterPlanner* ui_;
};

}  // namespace noether
