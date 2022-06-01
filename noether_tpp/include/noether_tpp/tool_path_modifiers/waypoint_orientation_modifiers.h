#pragma once

#include <noether_tpp/core/tool_path_modifier.h>

namespace noether
{
/**
 * @brief Aligns the orientation of each waypoint with the existing waypoint normal (z-axis) and the specified reference
 * x-axis direction
 * @details The new waypoint y-axis is computed as the cross-product of the existing waypoint normal (z-axis) and the
 * reference x-axis. The new waypoint x-axis is computed as the cross-product of the new y-axis with the existing
 * waypoint normal (z-axis). Thus, each new waypoint's x-axis will probably not exactly match the input reference
 * direction, but it will be as close as possible
 */
class FixedOrientationModifier : public OneTimeToolPathModifier
{
public:
  FixedOrientationModifier(const Eigen::Vector3d& reference_x_direction);
  ToolPaths modify(ToolPaths tool_paths) const override final;

protected:
  /** @brief Reference x-axis direction */
  const Eigen::Vector3d ref_x_dir_;
};

/**
 * @brief Aligns the x-axis of all waypoints with the direction of travel between adjacent waypoints
 */
struct DirectionOfTravelOrientationModifier : public OneTimeToolPathModifier
{
  ToolPaths modify(ToolPaths tool_paths) const override final;
};

/**
 * @brief Aligns the x-axis of all waypoints with the direction of travel of the first two waypoints
 * @details Re-alignment of the waypoint orientation only occurs if the dot product of the waypoint x-axis and the
 * reference direction of travel is less than zero. In the case of an orientation change, the waypoint is simply rotated
 * about its z-axis by 180 degrees
 */
struct UniformOrientationModifier : public OneTimeToolPathModifier
{
  ToolPaths modify(ToolPaths tool_paths) const override final;
};

/**
 * @brief Applies a moving average filter to waypoints in each tool path segment to smooth their orientations
 */
class MovingAverageOrientationSmoothingModifier : public ToolPathModifier
{
public:
  MovingAverageOrientationSmoothingModifier(std::size_t window_size);

  ToolPaths modify(ToolPaths tool_paths) const override;

protected:
  /** @brief Moving average window size **/
  const std::size_t window_size_;
};

}  // namespace noether
