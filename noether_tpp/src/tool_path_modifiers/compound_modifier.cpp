#include <noether_tpp/tool_path_modifiers/compound_modifier.h>

#include <utility>  // std::move()

namespace noether
{
CompoundModifier::CompoundModifier(std::vector<ToolPathModifier::ConstPtr> modifiers) : modifiers_(std::move(modifiers))
{
}

ToolPaths CompoundModifier::modify(ToolPaths paths) const
{
  // Apply the modifiers in order
  for (std::size_t i = 0; i < modifiers_.size(); ++i)
  {
    try
    {
      const auto& modifier = modifiers_[i];
      paths = modifier->modify(paths);
    }
    catch (const std::exception& ex)
    {
      std::stringstream ss;
      ss << "Error applying tool path modifier at index " << i << ".";
      std::throw_with_nested(std::runtime_error(ss.str()));
    }
  }
  return paths;
}

OneTimeCompoundModifier::OneTimeCompoundModifier(std::vector<OneTimeToolPathModifier::ConstPtr> modifiers)
  : modifiers_(std::move(modifiers))
{
}

ToolPaths OneTimeCompoundModifier::modify(ToolPaths paths) const
{
  // Apply the modifiers in order
  for (std::size_t i = 0; i < modifiers_.size(); ++i)
  {
    try
    {
      const auto& modifier = modifiers_[i];
      paths = modifier->modify(paths);
    }
    catch (const std::exception& ex)
    {
      std::stringstream ss;
      ss << "Error applying tool path modifier at index " << i << ".";
      std::throw_with_nested(std::runtime_error(ss.str()));
    }
  }
  return paths;
}

}  // namespace noether
