#include <noether_gui/widgets/tool_path_modifiers/snake_organization_modifier_widget.h>

namespace noether
{
void SnakeOrganizationModifierWidget::save(YAML::Node& config) const { config["name"] = "SnakeOrganization"; }

}  // namespace noether
