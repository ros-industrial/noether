#pragma once

#include <noether_tpp/core/tool_path_modifier.h>
#include <Eigen/Geometry>

namespace noether
{
/**
 * @ingroup tool_path_modifiers
 * @brief Offsets the tool path by a fixed pose
 */
class OffsetModifier : public ToolPathModifier
{
public:
  OffsetModifier(const Eigen::Isometry3d& offset);

  ToolPaths modify(ToolPaths tool_paths) const override;

protected:
  /** @brief fixed tool path offset */
  const Eigen::Isometry3d offset_;
};

}  // namespace noether
