#pragma once

#include <noether_gui/widgets.h>
#include <noether_tpp/core/tool_path_planner.h>

class QSpinBox;

namespace Ui
{
class Isometry3dEditor;
}

namespace Ui
{
class Vector2dEditor;
}

namespace noether
{
class FlatPlaneToolPathPlannerWidget : public ToolPathPlannerWidget
{
  Q_OBJECT
public:
  FlatPlaneToolPathPlannerWidget(QWidget* parent = nullptr);

  ToolPathPlanner::ConstPtr create() const override;

  void configure(const YAML::Node&) override;
  void save(YAML::Node&) const override;

private:
  Ui::Isometry3dEditor* origin_ui_;
  Ui::Vector2dEditor* plane_dim_ui_;
  Ui::Vector2dEditor* spacing_dim_ui_;
};

}  // namespace noether
