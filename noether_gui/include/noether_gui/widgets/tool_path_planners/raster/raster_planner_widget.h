#pragma once

#include <noether_gui/widgets.h>

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
/**
 * @ingroup gui_widgets_tool_path_planners
 */
class RasterPlannerWidget : public ToolPathPlannerWidget
{
public:
  RasterPlannerWidget(std::shared_ptr<const boost_plugin_loader::PluginLoader> loader, QWidget* parent = nullptr);

  void configure(const YAML::Node&) override;
  void save(YAML::Node&) const override;

protected:
  std::shared_ptr<const boost_plugin_loader::PluginLoader> loader_;
  DirectionGeneratorWidget* getDirectionGeneratorWidget() const;
  OriginGeneratorWidget* getOriginGeneratorWidget() const;

  Ui::RasterPlanner* ui_;
};

}  // namespace noether
