#include <noether_gui/widgets/tool_path_modifiers/direction_of_travel_orientation_modifier_widget.h>

#include <noether_tpp/tool_path_modifiers/direction_of_travel_orientation_modifier.h>

namespace noether
{
ToolPathModifier::ConstPtr DirectionOfTravelOrientationModifierWidget::create() const
{
  return std::make_unique<DirectionOfTravelOrientationModifier>();
}

}  // namespace noether
