#include <noether_tpp/tool_path_planners/edge/edge_planner.h>
#include <noether_tpp/tool_path_modifiers/standard_edge_paths_organization_modifier.h>
#include <noether_tpp/tool_path_modifiers/direction_of_travel_orientation_modifier.h>

namespace noether
{
ToolPaths EdgePlanner::plan(const pcl::PolygonMesh& mesh) const
{
  ToolPaths tool_paths = planImpl(mesh);
  if (tool_paths.empty())
    return tool_paths;

  // Apply the modifications necessary to produce the "default" behavior
  // First, organize the position of the waypoints into standard edge paths
  StandardEdgePathsOrganizationModifier edge_path_organizer;
  tool_paths = edge_path_organizer.modify(tool_paths);

  // Next, update the orientation of the waypoints such that their x-axes align with the direction of travel between
  // adjacent waypoints. Note: this modifier does not change the z-axis of the waypoints (normal to the surface)
  // returned by the planner implementation
  DirectionOfTravelOrientationModifier dot_orientation;
  tool_paths = dot_orientation.modify(tool_paths);

  return tool_paths;
}

}  // namespace noether
