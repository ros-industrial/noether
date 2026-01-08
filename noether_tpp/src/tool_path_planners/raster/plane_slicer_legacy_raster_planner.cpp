#include <noether_tpp/tool_path_planners/raster/plane_slicer_legacy_raster_planner.h>
#include <noether_tpp/tool_path_modifiers/compound_modifier.h>
#include <noether_tpp/tool_path_modifiers/join_close_segments_modifier.h>
#include <noether_tpp/tool_path_modifiers/minimum_segment_length_modifier.h>
#include <noether_tpp/tool_path_modifiers/uniform_spacing_modifier.h>
#include <noether_tpp/utils.h>

namespace noether
{
PlaneSlicerLegacyRasterPlanner::PlaneSlicerLegacyRasterPlanner(DirectionGenerator::ConstPtr dir_gen,
                                                               OriginGenerator::ConstPtr origin_gen,
                                                               const double point_spacing,
                                                               const double min_segment_size,
                                                               const double min_hole_size)
  : PlaneSlicerRasterPlanner(std::move(dir_gen), std::move(origin_gen))
  , point_spacing_(point_spacing)
  , min_segment_size_(min_segment_size)
  , min_hole_size_(min_hole_size)
{
}

ToolPaths PlaneSlicerLegacyRasterPlanner::planImpl(const pcl::PolygonMesh& mesh) const
{
  // Create a compound tool path modifier for 1) joining segments closer than the minimum hole size, 2) pruning segments
  // less than the minimum required length, and 3) uniformly linearly sampling waypoints on the tool path
  std::vector<ToolPathModifier::ConstPtr> mods(3);
  mods[0] = std::make_unique<JoinCloseSegmentsToolPathModifier>(min_hole_size_);
  mods[1] = std::make_unique<const MinimumSegmentLengthToolPathModifier>(min_segment_size_);
  mods[2] = std::make_unique<const UniformSpacingModifier>(point_spacing_, 1l, true);
  CompoundModifier mod(std::move(mods));

  return mod.modify(PlaneSlicerRasterPlanner::planImpl(mesh));
}

}  // namespace noether
