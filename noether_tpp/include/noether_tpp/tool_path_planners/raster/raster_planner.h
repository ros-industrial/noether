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

#include <memory>

#include <noether_tpp/core/tool_path_modifier.h>
#include <noether_tpp/core/tool_path_planner.h>

namespace noether
{
/**
 * @brief Interface for various methods to generate the direction of raster paths.  The direction
 * generated will move along the line from the first point to the second, then third, etc.
 */
struct DirectionGenerator
{
  virtual ~DirectionGenerator() = default;
  virtual Eigen::Vector3d generate(const pcl::PolygonMesh& mesh) const = 0;
};

/**
 * @brief Interface for setting the start point from which to generate toolpaths
 */
struct OriginGenerator
{
  virtual ~OriginGenerator() = default;
  virtual Eigen::Vector3d generate(const pcl::PolygonMesh& mesh) const = 0;
};

/**
 * @brief A specification of the tool path planner for covering surfaces
 * in repeating, evenly spaced path lines.  By default, a raster planner will produce a path with
 * all points having a consistent orientation, lines which all travel in the same direction across
 * the surface, and the traversal between lines will be in a consistent direction and order.
 */
class RasterPlanner : public ToolPathPlanner
{
public:
  RasterPlanner(std::unique_ptr<DirectionGenerator> dir_gen, std::unique_ptr<OriginGenerator> origin_gen);

  ToolPaths plan(const pcl::PolygonMesh& mesh) const override final;

protected:
  /**
   * @brief Implementation of the tool path planning capability
   * @param mesh
   * @return
   */
  virtual ToolPaths planImpl(const pcl::PolygonMesh& mesh) const = 0;

  std::unique_ptr<DirectionGenerator> dir_gen_;
  std::unique_ptr<OriginGenerator> origin_gen_;

private:
  /** @brief Distance between waypoints on the same raster line (m) */
  double point_spacing;
  /** @brief Distance between raster lines */
  double line_spacing;
  /** @brief Minimum size of hole in a mesh for which the planner should split a raster line that
   * crosses over the hole into multiple segments */
  double min_hole_size;
};

/**
 * @brief Interface for creating implementations of raster tool path planners.
 * @details This class contains the generic parameters for configuring raster tool path planners
 */
struct RasterPlannerFactory : public ToolPathPlannerFactory
{
  /** @brief Distance between waypoints on the same raster line (m) */
  double point_spacing;
  /** @brief Distance between raster lines */
  double line_spacing;
  /** @brief Minimum size of hole in a mesh for which the planner should split a raster line that
   * crosses over the hole into multiple segments */
  double min_hole_size;
};

}  // namespace noether
