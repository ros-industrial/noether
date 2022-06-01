#include <noether_tpp/tool_path_planners/raster/raster_planner.h>

#include <utility>  // std::move()

#include <noether_tpp/tool_path_modifiers/organization_modifiers.h>
#include <noether_tpp/tool_path_modifiers/waypoint_orientation_modifiers.h>

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

void RasterPlanner::setPointSpacing(const double point_spacing) { point_spacing_ = point_spacing; }
void RasterPlanner::setLineSpacing(const double line_spacing) { line_spacing_ = line_spacing; }
void RasterPlanner::setMinHoleSize(const double min_hole_size) { min_hole_size_ = min_hole_size; };

}  // namespace noether
