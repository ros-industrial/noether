#pragma once

#include <noether_gui/widgets.h>
#include <noether_tpp/core/tool_path_modifier.h>

namespace Ui
{
class Vector3dEditor;
class QuaternionEditor;
}  // namespace Ui

namespace noether
{
/**
 * @ingroup gui_widgets_tool_path_modifiers
 */
class OffsetModifierWidget : public ToolPathModifierWidget
{
public:
  OffsetModifierWidget(QWidget* parent = nullptr);
  ToolPathModifier::ConstPtr create() const override;

  void configure(const YAML::Node&) override;
  void save(YAML::Node&) const override;

private:
  Ui::Vector3dEditor* ui_vector_;
  Ui::QuaternionEditor* ui_quaternion_;
};

}  // namespace noether
