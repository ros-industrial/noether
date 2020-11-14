/**
 * @author Jorge Nicho <jrgnichodevel@gmail.com>
 * @file clean_data.h
 * @date Dec 6, 2019
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

#ifndef INCLUDE_NOETHER_FILTERING_MESH_CLEAN_DATA_H_
#define INCLUDE_NOETHER_FILTERING_MESH_CLEAN_DATA_H_

#include "noether_filtering/mesh/mesh_filter_base.h"

namespace noether_filtering
{
namespace mesh
{
/**
 * @class noether_filtering::mesh::CleanData
 * @details Uses the VTK capability shown in this example
 * https://lorensen.github.io/VTKExamples/site/Cxx/PolyData/CleanPolyData/ Merges duplicate points, and/or remove unused
 * points and/or remove degenerate cells
 */
class CleanData : public MeshFilterBase
{
public:
  CleanData();
  virtual ~CleanData();

  /**
   * @brief does not require configuration
   * @param config Here to conform to interface but can be an emtpy object
   * @return  True always
   */
  bool configure(XmlRpc::XmlRpcValue config) override final;

  /**
   * @brief Merges duplicate points, and/or remove unused points and/or remove degenerate cells
   * @param mesh_in
   * @param mesh_out
   * @return True always
   */
  bool filter(const pcl::PolygonMesh& mesh_in, pcl::PolygonMesh& mesh_out) override final;
  std::string getName() const override final;
};

} /* namespace mesh */
} /* namespace noether_filtering */

#endif /* INCLUDE_NOETHER_FILTERING_MESH_CLEAN_DATA_H_ */
