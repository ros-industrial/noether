#pragma once

#include <noether_tpp/core/tool_path_modifier.h>
#include <noether_tpp/macros.h>

FWD_DECLARE_YAML_STRUCTS()

namespace noether
{
/**
 * @brief Joins tool path segments that are within a specified Cartesian distance of one another
 * @ingroup tool_path_modifiers
 */
class JoinCloseSegmentsToolPathModifier : public OneTimeToolPathModifier
{
public:
  /**
   * @brief Constructor
   * @param distance Distance (m) below which adjacent tool path segments should be joined
   */
  JoinCloseSegmentsToolPathModifier(const double distance);

  ToolPaths modify(ToolPaths tool_paths) const override;

protected:
  JoinCloseSegmentsToolPathModifier() = default;
  DECLARE_YAML_FRIEND_CLASSES(JoinCloseSegmentsToolPathModifier);

  double distance_;
};

}  // namespace noether

FWD_DECLARE_YAML_CONVERT(noether::JoinCloseSegmentsToolPathModifier);
