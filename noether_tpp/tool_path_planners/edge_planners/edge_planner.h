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

#include <pcl/PolygonMesh.h>

#include <noether_tpp/core/tool_path_planner.h>
#include <noether_tpp/core/types.h>
#include <noether_tpp/tool_path_modifiers/default_edge_mod.h>

namespace noether_tpp
{

/**
 * @brief The EdgePlanner class - A specification of the tool path planner interface for generating
 * paths around the edges of surfaces.  Each edge planner will return a list of closed-loop edges.
 * Segments will be in sequential order in the ToolPath.  The ToolPaths (closed loops) are ordered
 * by descending length of the closed loops.  All loops will start near a specified point, and all
 * loops will process in the same direction.
 */
class EdgePlanner : public ToolPathPlanner
{
public:
private:
  virtual ToolPaths planImpl(const pcl::PolygonMesh& mesh) = 0;

  DefaultEdgeMod modifier_;
};

} // namespace noether_tpp
