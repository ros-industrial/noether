#include <noether_tpp/tool_path_modifiers/offset_modifier.h>
#include <noether_tpp/utils.h>

namespace noether
{
OffssetModifier::OffssetModifier(Eigen::Vector3d offset) : offset_(offset)
  : arc_angle_(arc_angle), arc_radius_(arc_radius), n_points_(n_points)
{
}

ToolPaths OffssetModifier::modify(ToolPaths tool_paths) const
{
    for (ToolPath& tp : tool_paths)
    {
        for (ToolPathSegment& tps : tp)
        {
            for (std::size_t i = 0; i < tps.size(); ++i)
            {
                tps[i].translation() += offset_;
            }
        }
    }
    
    return tool_paths;
}

}  // namespace noether
