#pragma once

#include <noether_gui/widgets.h>

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
class GuiFactory;

/**
 * @ingroup gui_widgets_tool_path_planners
 */
class RasterPlannerWidget : public BaseWidget
{
public:
  RasterPlannerWidget(std::shared_ptr<const GuiFactory> factory, QWidget* parent = nullptr);

  void configure(const YAML::Node&) override;
  void save(YAML::Node&) const override;

protected:
  BaseWidget* getDirectionGeneratorWidget() const;
  BaseWidget* getOriginGeneratorWidget() const;

  Ui::RasterPlanner* ui_;
};

}  // namespace noether
