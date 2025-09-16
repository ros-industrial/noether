#pragma once

#include <noether_gui/widgets.h>

namespace noether
{
struct AABBCenterOriginGeneratorWidget : public BaseWidget
{
public:
  using BaseWidget::BaseWidget;

  void save(YAML::Node&) const override;
};

}  // namespace noether
