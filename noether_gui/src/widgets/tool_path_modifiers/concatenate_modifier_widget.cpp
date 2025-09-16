#include <noether_gui/widgets/tool_path_modifiers/concatenate_modifier_widget.h>

namespace noether
{
void ConcatenateModifierWidget::save(YAML::Node& config) const { config["name"] = "Concatenate"; }

}  // namespace noether
