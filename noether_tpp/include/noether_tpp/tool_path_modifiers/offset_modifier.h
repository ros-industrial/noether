#pragma once

#include <noether_tpp/core/tool_path_modifier.h>
#include <Eigen/Geometry>

#include <noether_tpp/macros.h>

FWD_DECLARE_YAML_STRUCTS()

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
  Eigen::Isometry3d offset_;

  OffsetModifier() = default;
  DECLARE_YAML_FRIEND_CLASSES(OffsetModifier)
};

}  // namespace noether

FWD_DECLARE_YAML_CONVERT(noether::OffsetModifier)
