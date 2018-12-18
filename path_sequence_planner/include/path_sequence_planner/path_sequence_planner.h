/*
 * Copyright (c) 2016, Southwest Research Institute
 * All rights reserved.
 *
 */

#ifndef PATH_SEQUENCE_PLANNER_H
#define PATH_SEQUENCE_PLANNER_H

#include <tool_path_planner/tool_path_planner_base.h>

namespace path_sequence_planner
{
  class PathSequencePlanner
  {
  public:

    /**
     * @brief linkPaths Connects all of the paths_ into a single path and flips paths as necessary
     */
    virtual void linkPaths()=0;

    /**
     * @brief setPaths Sets the paths to be used for linking
     * @param paths The input set of paths
     */
    virtual void setPaths(std::vector<tool_path_planner::ProcessPath> paths)=0;

    /**
     * @brief getPaths Get the list of paths currently stored (some paths may be flipped after linking)
     * @return The set of paths currently stored
     */
    virtual std::vector<tool_path_planner::ProcessPath> getPaths()=0;

    /**
     * @brief getIndices Get the list of path indices denoting the order in which paths should be executed
     * @return The list path indices
     */
    virtual std::vector<int> getIndices()=0;
  };

}
#endif // PATH_SEQUENCE_PLANNER_H
