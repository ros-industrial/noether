#include <noether_gui/widgets/tool_path_modifiers/direction_of_travel_orientation_modifier_widget.h>

namespace noether
{
void DirectionOfTravelOrientationModifierWidget::save(YAML::Node& config) const
{
  config["name"] = "DirectionOfTravelOrientation";
}

}  // namespace noether
