/*
 * Copyright (c) 2016, Southwest Research Institute
 * All rights reserved.
 *
 */

#ifndef PATH_SEQUENCE_PLANNER_H
#define PATH_SEQUENCE_PLANNER_H

#include <tool_path_planner/tool_path_planner.h>

namespace path_sequence_planner
{
  class PathSequencePlanner
  {
  public:

    /**
     * @brief linkPaths Connects all of the paths_ into a single path and stores the results
     */
    void linkPaths();

    /**
     * @brief setPaths
     * @param paths
     */
    void setPaths(std::vector<tool_path_planner::ProcessPath> paths){paths_ = paths;}

    std::vector<tool_path_planner::ProcessPath> getPaths(){return paths_;}

    std::vector<int> getIndices(){return indices_;}

  private:
    std::vector<tool_path_planner::ProcessPath> paths_; /**< The input paths to operate on */
    std::vector<int> indices_;  /**< The list of indices specifying the order in which to execute the paths_ */
  };

}
#endif // PATH_SEQUENCE_PLANNER_H
