﻿#pragma once

#include <noether_tpp/core/tool_path_modifier.h>

namespace noether
{
/**
 * @brief Transforms the waypoints to correspond with the center of the ginding tool so that the edge
 * of the tool is in contact with the media
 */
class ToolDragOrientationToolPathModifier : public ToolPathModifier
{
public:
  ToolDragOrientationToolPathModifier(double angle_offset, double tool_radius);
  ToolPaths modify(ToolPaths tool_paths) const override final;

protected:
  const double angle_offset_;
  const double tool_radius_;
};

}  // namespace noether
