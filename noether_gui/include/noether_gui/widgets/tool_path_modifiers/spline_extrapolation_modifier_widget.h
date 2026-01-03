#pragma once

#include <noether_gui/widgets.h>

namespace Ui
{
class SplineExtrapolation;
}

namespace noether
{
/**
 * @ingroup gui_widgets_tool_path_modifiers
 */
struct SplineExtrapolationToolPathModifierWidget : public BaseWidget
{
public:
  SplineExtrapolationToolPathModifierWidget(QWidget* parent);

  void configure(const YAML::Node&) override;
  void save(YAML::Node&) const override;

protected:
  Ui::SplineExtrapolation* ui_;
};

}  // namespace noether
