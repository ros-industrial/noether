/**
 * @file edge_planner.h
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

#include <noether_tpp/core/tool_path_planner.h>

namespace noether
{
/**
 * @ingroup tool_path_planners
 * @brief A specification of the tool path planner interface for generating paths around the edges of surfaces
 * @details Each edge planner will return a list of closed-loop edges.
 * Segments will be in sequential order in the ToolPath.
 * The ToolPaths (closed loops) are ordered by descending length of the closed loops.
 * All loops will start near a specified point, and all loops will process in the same direction.
 */
class EdgePlanner : public ToolPathPlanner
{
public:
  ToolPaths plan(const pcl::PolygonMesh& mesh) const override final;

protected:
  /**
   * @brief Hook for inherited classes to implement the edge planning process before sending to the common edge tool
   * path modifiers
   */
  virtual ToolPaths planImpl(const pcl::PolygonMesh& mesh) const = 0;
};

/**
 * @brief Interface for creating instances of edge tool path planners.
 * @details This class contains the generic parameters for configuring edge tool path planners
 */
using EdgePlannerFactory = ToolPathPlannerFactory;

}  // namespace noether
