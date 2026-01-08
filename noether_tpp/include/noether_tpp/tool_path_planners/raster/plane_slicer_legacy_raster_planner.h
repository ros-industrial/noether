/**
 * @file plane_slicer_legacy_raster_planner.h
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

#include <noether_tpp/tool_path_planners/raster/plane_slicer_raster_planner.h>

namespace noether
{
/**
 * @ingroup raster_planners
 * @details Implementation of the legacy PlaneSlicerRasterPlanner, which includes control over point spacing, minimum
 * segment size, and minimum hole size.
 */
class PlaneSlicerLegacyRasterPlanner : public PlaneSlicerRasterPlanner
{
public:
  /**
   * @brief Constructor
   * @param dir_gen Direction generator
   * @param origin_gen Origin generator
   * @param point_spacing Distance between waypoints on the same raster line (m)
   * @param min_segment_size Minimum length of valid segment (m)
   * @param min_hole_size Minimum size of hole in a mesh for which the planner should split a raster line that
   * crosses over the hole into multiple segments
   */
  PlaneSlicerLegacyRasterPlanner(DirectionGenerator::ConstPtr dir_gen,
                                 OriginGenerator::ConstPtr origin_gen,
                                 const double point_spacing,
                                 const double min_segment_size,
                                 const double min_hole_size);

protected:
  ToolPaths planImpl(const pcl::PolygonMesh& mesh) const;

  double point_spacing_;
  double min_segment_size_;
  double min_hole_size_;
};

}  // namespace noether
