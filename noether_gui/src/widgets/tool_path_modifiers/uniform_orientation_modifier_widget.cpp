#include <noether_gui/widgets/tool_path_modifiers/uniform_orientation_modifier_widget.h>

#include <noether_tpp/tool_path_modifiers/uniform_orientation_modifier.h>

namespace noether
{
ToolPathModifier::ConstPtr UniformOrientationModifierWidget::create() const
{
  return std::make_unique<UniformOrientationModifier>();
}

}  // namespace noether
