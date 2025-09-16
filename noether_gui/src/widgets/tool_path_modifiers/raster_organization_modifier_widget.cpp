#include <noether_gui/widgets/tool_path_modifiers/raster_organization_modifier_widget.h>

namespace noether
{
void RasterOrganizationModifierWidget::save(YAML::Node& config) const { config["name"] = "RasterOrganization"; }

}  // namespace noether
