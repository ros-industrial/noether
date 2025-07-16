#pragma once

#include <noether_gui/widgets.h>

namespace Ui
{
class Vector3dEditor;
class LinearApproachModifier;
}  // namespace Ui

namespace noether
{
/**
 * @ingroup gui_widgets_tool_path_modifiers
 */
class LinearApproachToolPathModifierWidget : public ToolPathModifierWidget
{
public:
  LinearApproachToolPathModifierWidget(QWidget* parent = nullptr);

  void configure(const YAML::Node&) override;
  void save(YAML::Node&) const override;

protected:
  Ui::LinearApproachModifier* ui_;
  Ui::Vector3dEditor* vector_editor_ui_;
};

}  // namespace noether
