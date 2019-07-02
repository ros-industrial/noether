/**
 * @file simple_path_sequence_planner.h
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

#ifndef SIMPLE_PATH_SEQUENCE_PLANNER_H
#define SIMPLE_PATH_SEQUENCE_PLANNER_H

#include <path_sequence_planner/path_sequence_planner.h>

namespace path_sequence_planner
{
  class SimplePathSequencePlanner : public PathSequencePlanner
  {
  public:

    /**
     * @brief linkPaths Connects all of the paths_ into a single path and flips paths as necessary
     */
    void linkPaths();

    /**
     * @brief setPaths Sets the paths to be used for linking
     * @param paths The input set of paths
     */
    void setPaths(std::vector<tool_path_planner::ProcessPath> paths){paths_ = paths; indices_.clear();}

    /**
     * @brief getPaths Get the list of paths currently stored (some paths may be flipped after linking)
     * @return The set of paths currently stored
     */
    std::vector<tool_path_planner::ProcessPath> getPaths(){return paths_;}

    /**
     * @brief getIndices Get the list of path indices denoting the order in which paths should be executed
     * @return The list path indices
     */
    std::vector<int> getIndices(){return indices_;}

  private:

    /**
     * @brief findNextNearestPath Finds the next nearest path in a set to for a sequence
     * @param paths The set of paths to search
     * @param used_indices The list of indices already used
     * @param last_path The index of the current path to check for
     * @param front Used to determine whether to use the front or back of the last path to calculate distance
     * @return The index of the next nearest path to last_path
     */
    int findNextNearestPath(std::vector<tool_path_planner::ProcessPath> paths,
                             std::vector<int> used_indices, int last_path, bool front);

    std::vector<tool_path_planner::ProcessPath> paths_; /**< The input paths to operate on */
    std::vector<int> indices_;  /**< The list of indices specifying the order in which to execute the paths_ */
  };

}
#endif // PATH_SEQUENCE_PLANNER_H
