#include <noether_gui/widgets/tool_path_modifiers/raster_organization_modifier_widget.h>

#include <noether_tpp/tool_path_modifiers/raster_organization_modifier.h>

namespace noether
{
ToolPathModifier::ConstPtr RasterOrganizationModifierWidget::create() const
{
  return std::make_unique<RasterOrganizationModifier>();
}

}  // namespace noether
