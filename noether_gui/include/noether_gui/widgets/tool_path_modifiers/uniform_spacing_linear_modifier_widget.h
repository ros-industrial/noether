#pragma once

#include <noether_gui/widgets.h>

namespace Ui
{
class UniformSpacing;
}

namespace noether
{
/**
 * @ingroup gui_widgets_tool_path_modifiers
 */
class UniformSpacingLinearModifierWidget : public BaseWidget
{
public:
  UniformSpacingLinearModifierWidget(QWidget* parent = nullptr);

  void configure(const YAML::Node& config) override;
  void save(YAML::Node& config) const override;

protected:
  Ui::UniformSpacing* ui_;
};

}  // namespace noether
