#include <noether_gui/widgets/tool_path_modifiers/concatenate_modifier_widget.h>

#include <noether_tpp/tool_path_modifiers/concatenate_modifier.h>

namespace noether
{
ToolPathModifier::ConstPtr ConcatenateModifierWidget::create() const
{
  return std::make_unique<ConcatenateModifier>();
}

}  // namespace noether
