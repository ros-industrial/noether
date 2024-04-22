#include <noether_tpp/tool_path_modifiers/offset_modifier.h>
#include <noether_tpp/utils.h>

namespace noether
{
OffsetModifier::OffsetModifier(Eigen::Vector3d offset) : offset_(offset)
{
}

ToolPaths OffsetModifier::modify(ToolPaths tool_paths) const
{
    for (ToolPath& tp : tool_paths)
    {
        for (ToolPathSegment& tps : tp)
        {
            for (std::size_t i = 0; i < tps.size(); ++i)
            {
                tps[i] = tps[i] * Eigen::Translation3d(offset_);
            }
        }
    }
    
    return tool_paths;
}

}  // namespace noether
