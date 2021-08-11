#include <noether_tpp/tool_path_planners/raster/raster_planner.h>

#include <utility>  // std::move()

namespace noether
{
RasterPlanner::RasterPlanner(std::unique_ptr<DirectionGenerator> dir_gen, std::unique_ptr<OriginGenerator> origin_gen)
  : dir_gen_(std::move(dir_gen)), origin_gen_(std::move(origin_gen))
{
}

ToolPaths RasterPlanner::plan(const pcl::PolygonMesh& mesh) const
{
  ToolPaths tool_paths = planImpl(mesh);

  // To-do: implement the modifications necessary to produce the default behavior

  return tool_paths;
}

}  // namespace noether
