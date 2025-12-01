#include <noether_gui/widgets/tool_path_planners/raster/origin_generators/aabb_center_origin_generator_widget.h>

namespace noether
{
void AABBCenterOriginGeneratorWidget::save(YAML::Node& config) const { config["name"] = "AABBCenter"; }

}  // namespace noether
