#pragma once

#include <noether_tpp/core/tool_path_modifier.h>

namespace noether
{
/**
 * @ingroup tool_path_modifiers
 * @brief Transforms the waypoints to correspond with the edge of the ginding tool so that the edge
 * of the tool is in contact with the media
 */
class BiasedToolDragOrientationToolPathModifier : public ToolPathModifier
{
public:
  BiasedToolDragOrientationToolPathModifier(double angle_offset, double tool_radius);
  ToolPaths modify(ToolPaths tool_paths) const override final;

protected:
  const double angle_offset_;
  const double tool_radius_;
};

}  // namespace noether
