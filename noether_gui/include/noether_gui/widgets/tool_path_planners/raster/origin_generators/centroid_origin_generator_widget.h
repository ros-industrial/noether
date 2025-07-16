#pragma once

#include <noether_gui/widgets.h>

namespace noether
{
struct CentroidOriginGeneratorWidget : public OriginGeneratorWidget
{
public:
  using BaseWidget::BaseWidget;

  void save(YAML::Node&) const override;
};

}  // namespace noether
