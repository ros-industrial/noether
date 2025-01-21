/**
 * @file pipeline.h
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

#include <noether_tpp/core/mesh_modifier.h>
#include <noether_tpp/core/tool_path_modifier.h>
#include <noether_tpp/core/tool_path_planner.h>

#include <pcl/PolygonMesh.h>

namespace noether
{
/**
 * @details Collects together a set of mesh modifiers, a planner,
 * and a set of tool path modifiers. These sub-units then can be called using a single line to
 * pre-process meshes, plan paths, and operate on the paths.
 */
class ToolPathPlannerPipeline
{
public:
  ToolPathPlannerPipeline(MeshModifier::ConstPtr mesh_mod,
                          ToolPathPlanner::ConstPtr planner,
                          ToolPathModifier::ConstPtr tool_path_mod);

  std::vector<ToolPaths> plan(pcl::PolygonMesh mesh) const;
  MeshModifier::ConstPtr mesh_modifier;
  ToolPathPlanner::ConstPtr planner;
  ToolPathModifier::ConstPtr tool_path_modifier;
};

}  // namespace noether
