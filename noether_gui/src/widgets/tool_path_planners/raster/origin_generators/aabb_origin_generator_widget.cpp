#include <noether_gui/widgets/tool_path_planners/raster/origin_generators/aabb_origin_generator_widget.h>

namespace noether
{
void AABBOriginGeneratorWidget::save(YAML::Node& config) const { config["name"] = "AABBCenter"; }

}  // namespace noether
