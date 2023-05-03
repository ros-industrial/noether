#pragma once

#include <noether_gui/widgets.h>

#include <noether_tpp/tool_path_planners/raster/raster_planner.h>
#include <boost_plugin_loader/plugin_loader.h>

namespace Ui
{
class RasterPlanner;
}

namespace YAML
{
class Node;
}

namespace noether
{
class RasterPlannerWidget : public ToolPathPlannerWidget
{
public:
  RasterPlannerWidget(boost_plugin_loader::PluginLoader&& loader, QWidget* parent = nullptr);

  void configure(const YAML::Node&) override;
  void save(YAML::Node&) const override;

protected:
  DirectionGeneratorWidget* getDirectionGeneratorWidget() const;
  OriginGeneratorWidget* getOriginGeneratorWidget() const;

  void setDirectionGeneratorWidget(const QString& plugin_name, const YAML::Node& config);
  void setOriginGeneratorWidget(const QString& plugin_name, const YAML::Node& config);

  const boost_plugin_loader::PluginLoader loader_;
  Ui::RasterPlanner* ui_;
};

}  // namespace noether
