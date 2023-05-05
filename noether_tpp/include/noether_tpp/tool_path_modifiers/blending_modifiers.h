#pragma once

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

/**
 * @brief Modifier that adjusts the parameters of the tool approach trajectory to the media
 */
class CircularLeadInModifier : public ToolPathModifier
{
public:
  CircularLeadInModifier(double arc_angle, double arc_radius, double n_points);
  ToolPaths modify(ToolPaths tool_paths) const override final;

protected:
  const double arc_angle_;
  const double arc_radius_;
  const double n_points_;
};

/**
 * @brief Modifier that adds exit waypoints in a circular arc (with fixed orientation) to the end of a trajectory
 */
class CircularLeadOutModifier : public ToolPathModifier
{
public:
  CircularLeadOutModifier(double arc_angle, double arc_radius, double n_points);
  ToolPaths modify(ToolPaths tool_paths) const override final;

protected:
  const double arc_angle_;
  const double arc_radius_;
  const double n_points_;
};

}  // namespace noether
