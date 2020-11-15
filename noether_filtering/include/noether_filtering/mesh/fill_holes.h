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

#ifndef INCLUDE_NOETHER_FILTERING_MESH_FILL_HOLES_H_
#define INCLUDE_NOETHER_FILTERING_MESH_FILL_HOLES_H_

#include "noether_filtering/mesh/mesh_filter_base.h"

namespace noether_filtering
{
namespace mesh
{
/**
 * @class noether_filtering::mesh::FillHoles
 * @brief Applies the vtkFillHoles filter, more details can be found on
 *        https://vtk.org/doc/nightly/html/classvtkFillHolesFilter.html#details
 */
class FillHoles : public MeshFilterBase
{
public:
  FillHoles();
  virtual ~FillHoles();

  /**
   * @brief configures the fill hole algorithm from a yaml struct with the following form
   * - hole_size: 1.0 # Specify the maximum hole size to fill. Represented as a radius to the bounding circumsphere
   * containing the hole.
   *
   * @param config the configuration
   * @return  True on success, false otherwise
   */
  bool configure(XmlRpc::XmlRpcValue config) override final;

  /**
   * @brief Fills holes
   * @param mesh_in
   * @param mesh_out
   * @return True always
   */
  bool filter(const pcl::PolygonMesh& mesh_in, pcl::PolygonMesh& mesh_out) override final;
  std::string getName() const override final;

protected:
  double hole_size_ = 1.0;
};

} /* namespace mesh */
} /* namespace noether_filtering */

#endif /* INCLUDE_NOETHER_FILTERING_MESH_FILL_HOLES_H_ */
