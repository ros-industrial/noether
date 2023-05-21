#include <noether_gui/widgets/tool_path_modifiers/snake_organization_modifier_widget.h>

#include <noether_tpp/tool_path_modifiers/snake_organization_modifier.h>

namespace noether
{
ToolPathModifier::ConstPtr SnakeOrganizationModifierWidget::create() const
{
  return std::make_unique<SnakeOrganizationModifier>();
}

}  // namespace noether
