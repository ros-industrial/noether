#pragma once

#include <noether_tpp/core/tool_path_modifier.h>

namespace noether
{
/**
 * @brief Offsets the tool path by a fixed distance
 */
class OffsetModifier : public ToolPathModifier
{
public:
  OffsetModifier(Eigen::Vector3d offset);

  ToolPaths modify(ToolPaths tool_paths) const override;

protected:
  /** @brief In which direction to offset the tool path */
  const Eigen::Vector3d offset_;
};

}  // namespace noether
