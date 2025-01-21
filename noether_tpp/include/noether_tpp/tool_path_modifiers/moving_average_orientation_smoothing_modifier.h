#pragma once

#include <noether_tpp/core/tool_path_modifier.h>

namespace noether
{
/**
 * @ingroup tool_path_modifiers
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
