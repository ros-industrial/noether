/**
 * @file noether_conversions.h
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

#include <geometry_msgs/PoseArray.h>
#include <shape_msgs/Mesh.h>
#include <pcl/PolygonMesh.h>

namespace noether_conversions {

  /**
   * @brief Convenience conversion function
   * @param mesh_msg  The input mesh
   * @param mesh      The output pcl mesh
   * @return  True on success, false otherwise.
   */
  bool convertToPCLMesh(const shape_msgs::Mesh& mesh_msg, pcl::PolygonMesh& mesh);

  /**
   * @brief convenience conversion function
   * @param mesh      a polygon mesh from pcl
   * @param mesh_msg  a mesh message
   * @return True on success, false otherwise.
   */
  bool convertToMeshMsg(const pcl::PolygonMesh& mesh, shape_msgs::Mesh& mesh_msg);

  /**
   * @brief saves to a PLY file
   * @param filename
   * @param mesh_msg
   * @return True on success, false otherwise.
   */
  bool savePLYFile(const std::string& filename, const shape_msgs::Mesh& mesh_msg);

  /**
   * @brief loads from a PLY file
   * @param filename
   * @param mesh_msg
   * @return True on success, false otherwise.
   */
  bool loadPLYFile(const std::string& filename, shape_msgs::Mesh& mesh_msg);

}

