#include <noether_tpp/tool_path_modifiers/compound_modifier.h>

#include <utility>  // std::move()

namespace noether
{
CompoundModifier::CompoundModifier(std::vector<std::unique_ptr<const ToolPathModifier>> modifiers)
  : modifiers_(std::move(modifiers))
{
}

ToolPaths CompoundModifier::modify(ToolPaths paths) const
{
  // Apply the modifiers in order
  for (const auto& modifier : modifiers_)
  {
    paths = modifier->modify(paths);
  }
  return paths;
}

OneTimeCompoundModifier::OneTimeCompoundModifier(std::vector<std::unique_ptr<const OneTimeToolPathModifier>> modifiers)
  : modifiers_(std::move(modifiers))
{
}

ToolPaths OneTimeCompoundModifier::modify(ToolPaths paths) const
{
  // Apply the modifiers in order
  for (const auto& modifier : modifiers_)
  {
    paths = modifier->modify(paths);
  }
  return paths;
}

}  // namespace noether
