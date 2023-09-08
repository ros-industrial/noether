#pragma once

#include <noether_tpp/core/tool_path_modifier.h>

namespace noether
{
/**
 * @brief Modifier that adjusts the parameters of the tool approach trajectory to the media
 */
class LinearApproachModifier : public ToolPathModifier
{
public:
  enum class Axis
  {
    X,
    Y,
    Z,
  };

  LinearApproachModifier(Eigen::Vector3d offset, std::size_t n_points);
  LinearApproachModifier(Eigen::Vector3d offset, Axis axis, std::size_t n_points);
  ToolPaths modify(ToolPaths tool_paths) const override final;



protected:
  Eigen::Vector3d offset_;
  const size_t n_points_;
};

}  // namespace noether
