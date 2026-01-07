#include <noether_tpp/tool_path_planners/raster/raster_planner.h>
#include <noether_tpp/tool_path_modifiers/direction_of_travel_orientation_modifier.h>

#include <utility>  // std::move()

#include <noether_tpp/tool_path_modifiers/raster_organization_modifier.h>
#include <noether_tpp/tool_path_modifiers/fixed_orientation_modifier.h>

namespace noether
{
RasterPlanner::RasterPlanner(DirectionGenerator::ConstPtr dir_gen, OriginGenerator::ConstPtr origin_gen)
  : dir_gen_(std::move(dir_gen)), origin_gen_(std::move(origin_gen))
{
}

ToolPaths RasterPlanner::plan(const pcl::PolygonMesh& mesh) const
{
  // Call the raster planner implementation
  ToolPaths tool_paths = planImpl(mesh);
  if (tool_paths.empty())
    return tool_paths;

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

void RasterPlanner::setLineSpacing(const double line_spacing) { line_spacing_ = line_spacing; }

}  // namespace noether
