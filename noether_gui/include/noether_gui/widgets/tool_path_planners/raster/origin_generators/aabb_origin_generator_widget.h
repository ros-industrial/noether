#pragma once

#include <noether_gui/widgets.h>

namespace noether
{
struct AABBOriginGeneratorWidget : public OriginGeneratorWidget
{
public:
  using BaseWidget::BaseWidget;

  void save(YAML::Node&) const override;
};

}  // namespace noether
