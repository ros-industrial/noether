/**
 * @file raster_planner.h
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

#include <noether_tpp/core/raster_planner/direction_generator.h>
#include <noether_tpp/core/raster_planner/origin_generator.h>
#include <noether_tpp/core/tool_path_planner.h>
#include <noether_tpp/core/types.h>
#include <noether_tpp/tool_path_modifiers/default_raster_mod.h>

namespace noether_tpp
{

/**
 * @brief The RasterPlanner class - A specification of the tool path planner for covering surfaces
 * in repeating, evenly spaced path lines.  By default, a raster planner will produce a path with
 * all points having a consistent orientation, lines which all travel in the same direction across
 * the surface, and the traversal between lines will be in a consistent direction and order.
 */
class RasterPlanner : public ToolPathPlanner
{
public:
  ToolPaths planImpl(const pcl::PolygonMesh& mesh);

  DirectionGenerator dir_gen;
  OriginGenerator origin_gen;

private:
  virtual ToolPaths planImpl(const pcl::PolygonMesh& mesh) = 0;

  DefaultRasterMod modifier_;
};

} // namespace noether_tpp
