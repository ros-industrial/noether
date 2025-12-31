#pragma once

#include <noether_tpp/macros.h>
#include <noether_tpp/core/tool_path_modifier.h>

FWD_DECLARE_YAML_STRUCTS()

namespace noether
{
/**
 * @ingroup tool_path_modifiers
 * @brief Extrapolates the tool path
 * @details This modifier creates additional extrapolation waypoints at the front and/or back of a ToolPathSegment,
 * following the approximate curvature of the tool path (opposed to a linear offset). It works by fitting a spline to
 * the waypoints of each ToolPathSegment and extrapolating beyond the end of the spline using the last polynomial segment.
 * The extrapolated waypoint(s) can also include an optional offset along the waypoint normal (i.e., z-axis).
 */
class SplineExtrapolationToolPathModifier : public ToolPathModifier
{
public:
  /**
   * @brief Constructor
   * @param spline_degree Degree of the spline to fit to the tool path (e.g., 2 (parabolic), 3 (cubic))
   * @param extrapolation_distance Distance (m) beyond te to extrapolate the tool path
   * @param normal_offset_distance Offset distance (m) to apply along the extrapolated waypoint(s) normal (i.e., z-axis)
   * @param extrapolate_front Flag to indicate if the tool path should be extrapolated in the front (i.e., start)
   * @param extrapolate_back Flag to indicate if the tool path should be extrapolated in the back (i.e., end)
   */
  SplineExtrapolationToolPathModifier(const double spline_degree,
                                      const double extrapolation_distance,
                                      const double normal_offset_distance,
                                      const bool extrapolate_front,
                                      const bool extrapolate_back);

  ToolPaths modify(ToolPaths tool_path) const override;

protected:
  SplineExtrapolationToolPathModifier() = default;
  DECLARE_YAML_FRIEND_CLASSES(SplineExtrapolationToolPathModifier);

  double spline_degree_;
  double extrapolation_distance_;
  double normal_offset_distance_;
  bool extrapolate_front_;
  bool extrapolate_back_;
};

}  // namespace noether

FWD_DECLARE_YAML_CONVERT(noether::SplineExtrapolationToolPathModifier)
