#pragma once

#include <noether_tpp/macros.h>
#include <noether_tpp/core/tool_path_modifier.h>

FWD_DECLARE_YAML_STRUCTS()

namespace noether
{
class RadiusOfCurvatureExtrapolationToolPathModifier : public ToolPathModifier
{
public:
  RadiusOfCurvatureExtrapolationToolPathModifier(const double distance,
                                                 const double normal_offset_distance,
                                                 const bool extend_front,
                                                 const bool extend_back);

  ToolPaths modify(ToolPaths tool_path) const override;

protected:
  RadiusOfCurvatureExtrapolationToolPathModifier() = default;
  DECLARE_YAML_FRIEND_CLASSES(RadiusOfCurvatureExtrapolationToolPathModifier);

  double extrapolation_distance_;
  double normal_offset_distance_;
  bool extrapolate_front_;
  bool extrapolate_back_;
};

}  // namespace noether

FWD_DECLARE_YAML_CONVERT(noether::RadiusOfCurvatureExtrapolationToolPathModifier)
