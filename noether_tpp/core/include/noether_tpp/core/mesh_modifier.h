/**
 * @file mesh_modifier.h
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

#include <vector>

#include <pcl/PolygonMesh.h>

namespace noether
{
/**
 * @brief A common interface for mesh modifications.  Since some
 * modifications, such as segmentation, output multiple meshes, the function returns a list of
 * meshes.  Since the return value is a list, this interface (unlike the tool path modifier) takes
 * the input by reference.
 */
struct MeshModifier
{
  virtual ~MeshModifier() = default;
  virtual std::vector<pcl::PolygonMesh> modify(const pcl::PolygonMesh& mesh) const = 0;
};

}  // namespace noether
