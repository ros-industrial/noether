/**
 * @file mesh_segmenter.h
 * @copyright Copyright (c) 2019, Southwest Research Institute
 *
 * @author Jorge Nicho
 * @date Nov 15, 2018
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

#ifndef INCLUDE_TOOL_PATH_PLANNER_UTILITIES_H_
#define INCLUDE_TOOL_PATH_PLANNER_UTILITIES_H_

#include <noether_msgs/ToolPathConfig.h>
#include <Eigen/Dense>
#include <eigen_stl_containers/eigen_stl_containers.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <tool_path_planner/path_generator.h>
#include <cxxabi.h>

#include <tool_path_planner/halfedge_edge_generator.h>
#include <noether_msgs/HalfedgeEdgeGeneratorConfig.h>

#include <tool_path_planner/eigen_value_edge_generator.h>
#include <noether_msgs/EigenValueEdgeGeneratorConfig.h>

#include <tool_path_planner/surface_walk_raster_generator.h>
#include <noether_msgs/SurfaceWalkRasterGeneratorConfig.h>

#include <tool_path_planner/plane_slicer_raster_generator.h>
#include <noether_msgs/PlaneSlicerRasterGeneratorConfig.h>

namespace tool_path_planner
{
/**
 * @brief flipPointOrder Inverts a path
 * @param path The input path to invert
 */
void flipPointOrder(ToolPath& path);

/**
 * @brief conversion function. Not well tested yet
 * @param paths The toolpaths within a raster
 * @return A vector of tool path data
 */
ToolPathsData toToolPathsData(const ToolPaths& paths);

/**
 * @details creates a rotation matrix from the column vectors; it can then be assigned to a Isometry3d pose
 * as follows:
 * pose.matrix().block<3,3>(0,0) = toRotationMatrix(vx, vy, vz);
 * @param vx Column vector x
 * @param vy Column vector y
 * @param vz Column vector z
 * @return The rotation matrix
 */
Eigen::Matrix3d toRotationMatrix(const Eigen::Vector3d& vx, const Eigen::Vector3d& vy, const Eigen::Vector3d& vz);

bool toHalfedgeConfigMsg(noether_msgs::HalfedgeEdgeGeneratorConfig& config_msg,
                         const HalfedgeEdgeGenerator::Config& config);

bool toEigenValueConfigMsg(noether_msgs::EigenValueEdgeGeneratorConfig& config_msg,
                           const EigenValueEdgeGenerator::Config& config);

bool toSurfaceWalkConfigMsg(noether_msgs::SurfaceWalkRasterGeneratorConfig& config_msg,
                            const SurfaceWalkRasterGenerator::Config& config);

bool toPlaneSlicerConfigMsg(noether_msgs::PlaneSlicerRasterGeneratorConfig& config_msg,
                            const PlaneSlicerRasterGenerator::Config& config);

bool toHalfedgeConfig(HalfedgeEdgeGenerator::Config& config,
                      const noether_msgs::HalfedgeEdgeGeneratorConfig& config_msg);

bool toEigenValueConfig(EigenValueEdgeGenerator::Config& config,
                        const noether_msgs::EigenValueEdgeGeneratorConfig& config_msg);

bool toSurfaceWalkConfig(SurfaceWalkRasterGenerator::Config& config,
                         const noether_msgs::SurfaceWalkRasterGeneratorConfig& config_msg);

bool toPlaneSlicerConfig(PlaneSlicerRasterGenerator::Config& config,
                         const noether_msgs::PlaneSlicerRasterGeneratorConfig& config_msg);

/**
 * @details Creates an tool path segment from the point cloud with normals
 * It treats the point cloud as a set of consecutive points and so the x direction is along the line that
 * connects the current point to the next in the sequence .  The y vector is obtained from the cross product
 *  of the point normal and the x direction.
 * @param cloud_normals The points with normals
 * @param indices       Selects only these points, if left empty all points are used.
 * @segment             The output segment
 * @return  True on success, false otherwise.
 */
bool createToolPathSegment(const pcl::PointCloud<pcl::PointNormal>& cloud_normals,
                           const std::vector<int>& indices,
                           ToolPathSegment& segment);

template <class C>
static std::string getClassName()
{
  int status = -4;  // some arbitrary value to eliminate the compiler warning
  const char* mangled_name = typeid(C).name();

  // enable c++11 by passing the flag -std=c++11 to g++
  std::unique_ptr<char, void (*)(void*)> res{ abi::__cxa_demangle(mangled_name, NULL, NULL, &status), std::free };
  return (status == 0) ? res.get() : mangled_name;
}

ToolPaths splitSegments(ToolPaths tool_paths, double max_segment_length);

}  // namespace tool_path_planner

#endif /* INCLUDE_TOOL_PATH_PLANNER_UTILITIES_H_ */
