#include <noether_gui/widgets/tool_path_planners/raster/origin_generators/aabb_origin_generator_widget.h>

#include <noether_tpp/tool_path_planners/raster/origin_generators/aabb_origin_generator.h>

namespace noether
{
OriginGenerator::ConstPtr AABBOriginGeneratorWidget::create() const
{
  return std::make_unique<AABBCenterOriginGenerator>();
}

}  // namespace noether
