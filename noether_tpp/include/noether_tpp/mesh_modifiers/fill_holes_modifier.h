/**
 * @author Jorge Nicho <jrgnichodevel@gmail.com>
 * @file fill_holes.h
 * @date Dec 16, 2019
 * @copyright Copyright (c) 2019, Southwest Research Institute
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

namespace noether
{
/**
 * @ingroup mesh_modifiers
 * @brief Fills holes in the mesh using `vtkFillHoles`
 * @sa https://vtk.org/doc/nightly/html/classvtkFillHolesFilter.html#details
 */
class FillHoles : public MeshModifier
{
public:
  FillHoles(const double max_hole_size);

  std::vector<pcl::PolygonMesh> modify(const pcl::PolygonMesh& mesh) const override;

protected:
  double max_hole_size_;
};

}  // namespace noether
