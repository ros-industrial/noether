#pragma once

#include <noether_tpp/core/tool_path_modifier.h>

namespace noether
{
/**
 * @brief Transforms the waypoints to correspond with the center of the ginding tool so that the edge
 * of the tool is in contact with the media
*/
class AngledOrientationModifier : public ToolPathModifier
{
public:
  AngledOrientationModifier(double angle_offset, double tool_radius, bool flip_sign);
  ToolPaths modify(ToolPaths tool_paths) const override final;

protected:
  const double angle_offset_;
  const double tool_radius_;
  const bool flip_sign_;
};

/**
 * @brief Modifier that adjusts the parameters of the tool approach trajectory to the media
 * @details Asumes that the x direction of the waypoints is the direction of travel for the tool.
 * If this is not a valid assumption, run the DirectionOfTravel orientation modifier before this modifier.
*/
class LeadInModifier : public ToolPathModifier
{
public:
  LeadInModifier(double arc_angle, double arc_radius, double n_points);
  ToolPaths modify(ToolPaths tool_paths) const override final;

protected:
  const double arc_angle_;
  const double arc_radius_;
  const double n_points_;
};

/**
 * @brief Modifier that adjusts the parameters of the tool exit trajectory to the media
 * @details Asumes that the x direction of the waypoints is the direction of travel for the tool.
 * If this is not a valid assumption, run the DirectionOfTravel orientation modifier before this modifier.
*/
class LeadOutModifier : public ToolPathModifier
{
public:
  LeadOutModifier(double arc_angle, double arc_radius, double n_points);
  ToolPaths modify(ToolPaths tool_paths) const override final;

protected:
  const double arc_angle_;
  const double arc_radius_;
  const double n_points_;
};

}  // namespace noether

