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
 * the waypoints of each ToolPathSegment and extrapolating beyond the end of the spline using the last polynomial
 * segment. The extrapolated waypoint(s) can also include an optional offset along the waypoint normal (i.e., z-axis).
 */
class SplineExtrapolationToolPathModifier : public ToolPathModifier
{
public:
  /**
   * @brief Constructor
   * @param spline_degree Degree of the spline to fit to the tool path (e.g., 2 (parabolic), 3 (cubic))
   * @param extrapolation_distance_front Distance (m) in front of the spline to extrapolate the tool path
   * @param normal_offset_distance_front Offset distance (m) to apply along the front extrapolated waypoint(s) normal
   * (i.e., z-axis)
   * @param extrapolation_distance_back Distance (m) beyond the end of the spline to extrapolate the tool path
   * @param normal_offset_distance_back Offset distance (m) to apply along the back extrapolated waypoint(s) normal
   * (i.e., z-axis)
   */
  SplineExtrapolationToolPathModifier(const double spline_degree,
                                      const double extrapolation_distance_front,
                                      const double normal_offset_distance_front,
                                      const double extrapolation_distance_back,
                                      const double normal_offset_distance_back);

  ToolPaths modify(ToolPaths tool_path) const override;

protected:
  SplineExtrapolationToolPathModifier() = default;
  DECLARE_YAML_FRIEND_CLASSES(SplineExtrapolationToolPathModifier);

  double spline_degree_;
  double extrapolation_distance_front_;
  double normal_offset_distance_front_;
  double extrapolation_distance_back_;
  double normal_offset_distance_back_;
};

}  // namespace noether

FWD_DECLARE_YAML_CONVERT(noether::SplineExtrapolationToolPathModifier)
