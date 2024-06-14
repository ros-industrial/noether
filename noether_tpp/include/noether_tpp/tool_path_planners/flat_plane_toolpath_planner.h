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

#include <noether_tpp/core/tool_path_planner.h>
#include <string.h>

namespace noether
{
/**
 * @brief An implementation of a tool path planner that generates a toolpath on a flat plane.
 */
class FlatPlaneToolPathPlanner : public ToolPathPlanner
{
public:
  FlatPlaneToolPathPlanner(const Eigen::Vector2d& plane_dims,
                           const Eigen::Vector2d& point_spacing,
                           Eigen::Isometry3d offset = Eigen::Isometry3d::Identity());
  ToolPaths plan(const pcl::PolygonMesh& mesh) const override final;

private:
  const Eigen::Vector2d plane_dims_;
  const Eigen::Vector2d point_spacing_;
  const Eigen::Isometry3d offset_;
};

}  // namespace noether
