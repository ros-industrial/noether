#pragma once

#include <noether_gui/widgets.h>

namespace noether
{
class DistanceDoubleSpinBox;

/**
 * @ingroup gui_widgets_tool_path_modifiers
 */
class JoinCloseSegmentsToolPathModifierWidget : public BaseWidget
{
public:
  JoinCloseSegmentsToolPathModifierWidget(QWidget* parent = nullptr);

  void configure(const YAML::Node& config) override;
  void save(YAML::Node& config) const override;

protected:
  DistanceDoubleSpinBox* distance_;
};

}  // namespace noether
