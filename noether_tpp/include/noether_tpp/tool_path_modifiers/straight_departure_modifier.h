#pragma once

#include <noether_tpp/core/tool_path_modifier.h>

namespace noether
{
/**
 * @brief Modifier that adjusts the parameters of the tool departure trajectory to the media
 */
class StraightDepartureModifier : public ToolPathModifier
{
public:
  StraightDepartureModifier(double offset_height_, double n_points);
  ToolPaths modify(ToolPaths tool_paths) const override final;

protected:
  const double offset_height_;
  const double n_points_;
};

}  // namespace noether

