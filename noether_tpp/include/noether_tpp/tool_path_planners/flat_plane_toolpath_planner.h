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
#include <noether_tpp/macros.h>

FWD_DECLARE_YAML_STRUCTS()

namespace noether
{
/**
 * @brief An implementation of a tool path planner that generates a toolpath on a flat plane.
 */
class FlatPlaneToolPathPlanner : public ToolPathPlanner
{
public:
  FlatPlaneToolPathPlanner(double x_dim, double y_dim, double x_spacing, double y_spacing);
  ToolPaths plan(const pcl::PolygonMesh& mesh) const override final;

protected:
  double x_dim_;
  double y_dim_;
  double x_spacing_;
  double y_spacing_;

  FlatPlaneToolPathPlanner() = default;
  DECLARE_YAML_FRIEND_CLASSES(FlatPlaneToolPathPlanner)
};

}  // namespace noether

FWD_DECLARE_YAML_CONVERT(noether::FlatPlaneToolPathPlanner)
