#include <noether_gui/widgets/tool_path_modifiers/uniform_orientation_modifier_widget.h>

namespace noether
{
void UniformOrientationModifierWidget::save(YAML::Node& config) const { config["name"] = "UniformOrientation"; }

}  // namespace noether
