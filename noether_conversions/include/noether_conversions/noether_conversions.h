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
#include <visualization_msgs/MarkerArray.h>
#include <noether_msgs/ToolPaths.h>
#include <shape_msgs/Mesh.h>
#include <pcl/point_cloud.h>
#include <pcl/PolygonMesh.h>
#include <pcl/point_types.h>

namespace noether_conversions
{
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
 * @param filename    The path where to save the file
 * @param mesh_msg    The mesh
 * @param precision   The numerical precision
 * @param binary      Set to true to save in binary format
 * @return True on success, false otherwise.
 */
bool savePLYFile(const std::string& filename,
                 const shape_msgs::Mesh& mesh_msg,
                 unsigned precision = 10,
                 bool binary = true);

/**
 * @brief loads from a PLY file
 * @param filename
 * @param mesh_msg
 * @return True on success, false otherwise.
 */
bool loadPLYFile(const std::string& filename, shape_msgs::Mesh& mesh_msg);

visualization_msgs::Marker createMeshMarker(const std::string& mesh_file,
                                            const std::string& ns,
                                            const std::string& frame_id,
                                            const std::tuple<double, double, double, double>& rgba);

void convertToPointNormals(const pcl::PolygonMesh& mesh,
                           pcl::PointCloud<pcl::PointNormal>& cloud,
                           bool flip = false,
                           bool silent = true);

visualization_msgs::MarkerArray convertToAxisMarkers(
    const noether_msgs::ToolPaths& toolpaths,
    const std::string& frame_id,
    const std::string& ns,
    const std::size_t& start_id = 1,
    const double& axis_scale = 0.001,
    const double& axis_length = 0.03,
    const std::tuple<float, float, float, float, float, float>& offset = std::make_tuple(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));

visualization_msgs::MarkerArray convertToArrowMarkers(
    const noether_msgs::ToolPaths& toolpaths,
    const std::string& frame_id,
    const std::string& ns,
    const std::size_t start_id = 1,
    const float arrow_diameter = 0.002,
    const float point_size = 0.01,
    const std::tuple<float, float, float, float, float, float>& offset = std::make_tuple(0.0, 0.0, 0.0, 0.0, 0.0, 0.0));

visualization_msgs::MarkerArray convertToDottedLineMarker(
    const noether_msgs::ToolPaths& toolpaths,
    const std::string& frame_id,
    const std::string& ns,
    const std::size_t& start_id = 1,
    const std::tuple<float, float, float, float, float, float>& offset = std::make_tuple(0.0, 0.0, 0.0, 0.0, 0.0, 0.0),
    const float& line_width = 0.001f,
    const float& point_size = 0.005f);

}  // namespace noether_conversions
