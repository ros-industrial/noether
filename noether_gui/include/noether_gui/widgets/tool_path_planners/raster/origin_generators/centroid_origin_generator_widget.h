#pragma once

#include <noether_gui/widgets.h>

#include <noether_tpp/tool_path_planners/raster/raster_planner.h>

namespace noether
{
struct CentroidOriginGeneratorWidget : public OriginGeneratorWidget
{
  Q_OBJECT
public:
  using BaseWidget::BaseWidget;

  OriginGenerator::ConstPtr create() const override;
};

}  // namespace noether
