#pragma once

#include <noether_tpp/core/tool_path_modifier.h>
#include <noether_tpp/macros.h>

FWD_DECLARE_YAML_STRUCTS()

namespace noether
{
/**
 * @brief Prunes tool path segments whose lengths are less than the specified minimum length
 * @ingroup tool_path_modifiers
 */
class MinimumSegmentLengthToolPathModifier : public OneTimeToolPathModifier
{
public:
  /**
   * @brief Constructor
   * @param minimum_length Minimum length (m) of a tool path segment
   */
  MinimumSegmentLengthToolPathModifier(const double minimum_length);

  ToolPaths modify(ToolPaths tool_paths) const override;

protected:
  MinimumSegmentLengthToolPathModifier() = default;
  DECLARE_YAML_FRIEND_CLASSES(MinimumSegmentLengthToolPathModifier);

  double minimum_length_;
};

}  // namespace noether

FWD_DECLARE_YAML_CONVERT(noether::MinimumSegmentLengthToolPathModifier);
