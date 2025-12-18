#pragma once

#include <noether_gui/widgets.h>

namespace Ui
{
class RadiusOfCurvatureExtrapolation;
}

namespace noether
{
/**
 * @ingroup gui_widgets_tool_path_modifiers
 */
struct RadiusOfCurvatureExtrapolationToolPathModifierWidget : public BaseWidget
{
public:
  RadiusOfCurvatureExtrapolationToolPathModifierWidget(QWidget* parent);

  void configure(const YAML::Node&) override;
  void save(YAML::Node&) const override;

protected:
  Ui::RadiusOfCurvatureExtrapolation* ui_;
};

}  // namespace noether
