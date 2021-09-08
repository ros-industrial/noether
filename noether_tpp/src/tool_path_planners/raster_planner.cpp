#include <noether_tpp/tool_path_planners/raster/raster_planner.h>

#include <utility>  // std::move()

#include <noether_tpp/tool_path_modifiers/organization_modifiers.h>
#include <noether_tpp/tool_path_modifiers/waypoint_orientation_modifiers.h>

namespace noether
{
RasterPlanner::RasterPlanner(std::unique_ptr<DirectionGenerator> dir_gen, std::unique_ptr<OriginGenerator> origin_gen)
  : dir_gen_(std::move(dir_gen)), origin_gen_(std::move(origin_gen))
{
}

ToolPaths RasterPlanner::plan(const pcl::PolygonMesh& mesh) const
{
  // Call the raster planner implementation
  ToolPaths tool_paths = planImpl(mesh);

  // Apply the modifications necessary to produce the "default" behavior
  // First, organize the position of the waypoints into a raster pattern
  RasterOrganizationModifier raster;
  tool_paths = raster.modify(tool_paths);

  // Next, update the orientation of the waypoints such that their x-axes align with the direction of travel between
  // adjacent waypoints. Note: this modifier does not change the z-axis of the waypoints (normal to the surface)
  // returned by the planner implementation
  DirectionOfTravelOrientationModifier dot_orientation;
  tool_paths = dot_orientation.modify(tool_paths);

  return tool_paths;
}

}  // namespace noether
