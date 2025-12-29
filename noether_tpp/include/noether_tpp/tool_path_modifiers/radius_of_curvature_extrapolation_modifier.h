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
 * the waypoints of each ToolPathSegment and using that spline to compute the radius of curvature at each endpoint. It
 * then extrapolates the tool path along the radius of curvature at the endpoints and inserts a waypoint at the front
 * and/or back of the segment. The extrapolated waypoint(s) can also include an optional offset along the waypoint
 * normal (i.e., z-axis).
 */
class RadiusOfCurvatureExtrapolationToolPathModifier : public ToolPathModifier
{
public:
  /**
   * @brief Constructor
   * @param extrapolation_distance Arc distance (m) to extrapolate the tool path
   * @param normal_offset_distance Offset distance (m) to apply along the extrapolated waypoint(s) normal (i.e., z-axis)
   * @param extrapolate_front Flag to indicate if the tool path should be extrapolated in the front (i.e., start)
   * @param extrapolate_back Flag to indicate if the tool path should be extrapolated in the back (i.e., end)
   */
  RadiusOfCurvatureExtrapolationToolPathModifier(const double extrapolation_distance,
                                                 const double normal_offset_distance,
                                                 const bool extrapolate_front,
                                                 const bool extrapolate_back);

  ToolPaths modify(ToolPaths tool_path) const override;

protected:
  RadiusOfCurvatureExtrapolationToolPathModifier() = default;
  DECLARE_YAML_FRIEND_CLASSES(RadiusOfCurvatureExtrapolationToolPathModifier);

  double extrapolation_distance_;
  double normal_offset_distance_;
  bool extrapolate_front_;
  bool extrapolate_back_;
};

}  // namespace noether

FWD_DECLARE_YAML_CONVERT(noether::RadiusOfCurvatureExtrapolationToolPathModifier)
