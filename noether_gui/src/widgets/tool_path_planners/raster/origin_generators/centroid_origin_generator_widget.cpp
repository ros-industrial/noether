#include <noether_gui/widgets/tool_path_planners/raster/origin_generators/centroid_origin_generator_widget.h>

namespace noether
{
void CentroidOriginGeneratorWidget::save(YAML::Node& config) const { config["name"] = "Centroid"; }

}  // namespace noether
