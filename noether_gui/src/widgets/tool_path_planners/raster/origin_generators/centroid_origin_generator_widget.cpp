#include <noether_gui/widgets/tool_path_planners/raster/origin_generators/centroid_origin_generator_widget.h>

#include <noether_tpp/tool_path_planners/raster/origin_generators/centroid_origin_generator.h>

namespace noether
{
OriginGenerator::ConstPtr CentroidOriginGeneratorWidget::create() const
{
  return std::make_unique<CentroidOriginGenerator>();
}

}  // namespace noether
