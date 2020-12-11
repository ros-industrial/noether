/**
 * @file path_sequence_planner.h
 * @copyright Copyright (c) 2019, Southwest Research Institute
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

#ifndef PATH_SEQUENCE_PLANNER_H
#define PATH_SEQUENCE_PLANNER_H

#include <tool_path_planner/path_generator.h>

namespace path_sequence_planner
{
class PathSequencePlanner
{
public:
  PathSequencePlanner() = default;
  virtual ~PathSequencePlanner() = default;

  /** @brief linkPaths Connects all of the paths_ into a single path and flips paths as necessary */
  virtual void linkPaths() = 0;

  /**
   * @brief setPaths Sets the paths to be used for linking
   * @param paths The input set of paths
   */
  virtual void setPaths(tool_path_planner::ToolPaths paths) = 0;

  /**
   * @brief getPaths Get the list of paths currently stored (some paths may be flipped after linking)
   * @return The set of paths currently stored
   */
  virtual tool_path_planner::ToolPaths getPaths() const = 0;

  /**
   * @brief getIndices Get the list of path indices denoting the order in which paths should be executed
   * @return The list path indices
   */
  virtual std::vector<std::size_t> getIndices() const = 0;
};

}  // namespace path_sequence_planner
#endif  // PATH_SEQUENCE_PLANNER_H
