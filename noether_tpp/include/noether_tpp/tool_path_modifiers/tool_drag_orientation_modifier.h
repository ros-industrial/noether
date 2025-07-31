#pragma once

#include <noether_tpp/core/tool_path_modifier.h>
#include <noether_tpp/macros.h>

FWD_DECLARE_YAML_STRUCTS()

namespace noether
{
/**
 * @ingroup tool_path_modifiers
 * @brief Transforms the waypoints to correspond with the center of the ginding tool so that the edge
 * of the tool is in contact with the media
 */
class ToolDragOrientationToolPathModifier : public ToolPathModifier
{
public:
  ToolDragOrientationToolPathModifier(double angle_offset, double tool_radius);
  ToolPaths modify(ToolPaths tool_paths) const override final;

protected:
  double angle_offset_;
  double tool_radius_;

  ToolDragOrientationToolPathModifier() = default;
  DECLARE_YAML_FRIEND_CLASSES(ToolDragOrientationToolPathModifier)
};

}  // namespace noether

FWD_DECLARE_YAML_CONVERT(noether::ToolDragOrientationToolPathModifier)
