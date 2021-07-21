#include <noether_tpp/tool_path_planners/raster/raster_planner.h>

namespace noether_tpp
{
RasterPlanner::RasterPlanner(std::unique_ptr<DirectionGenerator>&& dir_gen,
                             std::unique_ptr<OriginGenerator>&& origin_gen)
  : dir_gen_(std::move(dir_gen)), origin_gen_(std::move(origin_gen)), modifier_(std::move(createDefaultModifier()))
{
}

std::unique_ptr<const OneTimeToolPathModifier> RasterPlanner::createDefaultModifier() { return nullptr; }

ToolPaths RasterPlanner::plan(const pcl::PolygonMesh& mesh) const { return modifier_->modify(planImpl(mesh)); }

}  // namespace noether_tpp
