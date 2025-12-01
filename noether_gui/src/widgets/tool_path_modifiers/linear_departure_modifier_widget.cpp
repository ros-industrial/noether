#include <noether_gui/widgets/tool_path_modifiers/linear_departure_modifier_widget.h>

namespace noether
{
void LinearDepartureToolPathModifierWidget::save(YAML::Node& config) const
{
  LinearApproachToolPathModifierWidget::save(config);
  config["name"] = "LinearDeparture";
}

}  // namespace noether
