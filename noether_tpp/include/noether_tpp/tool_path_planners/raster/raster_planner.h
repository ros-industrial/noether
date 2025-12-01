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
 * @ingroup direction_generators
 * @brief Interface for generating the direction of raster paths.
 * @details This interface only defines the raster path direction.
 * This direction represents the line along which waypoints in a raster path will lie.
 * It is overconstrained to also define the raster step direction, which is typically normal to the raster path
 * direction. As such, we leave it up to the individual raster planners to determine how propagate sequential rasters
 * normal to the path direction.
 */
struct DirectionGenerator
{
  using Ptr = std::unique_ptr<DirectionGenerator>;
  using ConstPtr = std::unique_ptr<const DirectionGenerator>;

  virtual ~DirectionGenerator() = default;
  virtual Eigen::Vector3d generate(const pcl::PolygonMesh& mesh) const = 0;
};

/**
 * @ingroup origin_generators
 * @brief Interface for setting the start point from which to generate toolpaths
 */
struct OriginGenerator
{
  using Ptr = std::unique_ptr<OriginGenerator>;
  using ConstPtr = std::unique_ptr<const OriginGenerator>;

  virtual ~OriginGenerator() = default;
  virtual Eigen::Vector3d generate(const pcl::PolygonMesh& mesh) const = 0;
};

/**
 * @ingroup tool_path_planners
 * @brief A specification of the tool path planner for covering surfaces in repeating, evenly spaced path lines.
 * @details By default, a raster planner will produce a path with
 * all points having a consistent orientation, lines which all travel in the same direction across
 * the surface, and the traversal between lines will be in a consistent direction and order.
 */
class RasterPlanner : public ToolPathPlanner
{
public:
  RasterPlanner(DirectionGenerator::ConstPtr dir_gen, OriginGenerator::ConstPtr origin_gen);

  ToolPaths plan(const pcl::PolygonMesh& mesh) const override final;

  void setPointSpacing(const double point_spacing);
  void setLineSpacing(const double line_spacing);
  void setMinHoleSize(const double min_hole_size);

protected:
  /**
   * @brief Implementation of the tool path planning capability
   */
  virtual ToolPaths planImpl(const pcl::PolygonMesh& mesh) const = 0;

  DirectionGenerator::ConstPtr dir_gen_;
  OriginGenerator::ConstPtr origin_gen_;

  /** @brief Distance between waypoints on the same raster line (m) */
  double point_spacing_;
  /** @brief Distance between raster lines */
  double line_spacing_;
  /** @brief Minimum size of hole in a mesh for which the planner should split a raster line that
   * crosses over the hole into multiple segments */
  double min_hole_size_;
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
  /** @brief Function for creating a raster direction generator */
  std::function<std::unique_ptr<const DirectionGenerator>()> direction_gen;
  /** @brief Function for creating a raster origin generator */
  std::function<std::unique_ptr<const OriginGenerator>()> origin_gen;
};

}  // namespace noether
