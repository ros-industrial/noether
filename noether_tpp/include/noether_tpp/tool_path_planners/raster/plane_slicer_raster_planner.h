/**
 * @file plane_slicer_raster_planner.h
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

#include <noether_tpp/tool_path_planners/raster/raster_planner.h>

namespace noether
{
/**
 * @brief An implementation of the Raster Planner using a series of parallel cutting planes.
 * This implementation works best on approximately planar parts.
 */
class PlaneSlicerRasterPlanner : public RasterPlanner
{
public:
  PlaneSlicerRasterPlanner(DirectionGenerator::ConstPtr dir_gen, OriginGenerator::ConstPtr origin_gen);

  void setSearchRadius(const double search_radius);
  void setMinSegmentSize(const double min_segment_size);

protected:
  /**
   * @brief Implementation of the tool path planning capability
   * @param mesh
   * @return
   */
  ToolPaths planImpl(const pcl::PolygonMesh& mesh) const;

private:
  /** @brief Minimum length of valid segment (m) */
  double min_segment_size_;
  /** @brief Search radius for calculating normals (m) */
  double search_radius_;
};

}  // namespace noether
