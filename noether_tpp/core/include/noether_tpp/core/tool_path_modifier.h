/**
 * @file tool_path_modifier.h
 * @copyright Copyright (c) 2021, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#pragma once

#include <noether_tpp/core/types.h>

namespace noether_tpp
{
/**
 * @brief A common interface for functions that alter a generated
 * toolpath.  Modifiers using this interface may or may not cause additional changes on repeated
 * use.  A special interface exists for modifiers that do not cause additional changes on repeat.
 * Since a tool path modifier, by definition, modifies a path, a ToolPaths is passed by value.
 */
class ToolPathModifier
{
public:
  virtual ~ToolPathModifier() = default;
  virtual ToolPaths modify(ToolPaths toolpaths) const = 0;
};

/**
 * @brief An extension of the ToolPathModifier interface that
 * requires a modifier to not cause additional changes if run again.  This will be enforced via
 * unit test.  New modifiers should be implemented as OneTimeToolPathModifiers if possible.
 */
class OneTimeToolPathModifier : public ToolPathModifier
{
public:
  virtual ToolPaths modify(ToolPaths tool_paths) const = 0;
};

}  // namespace noether_tpp
