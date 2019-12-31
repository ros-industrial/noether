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
#include <noether_msgs/ToolRasterPath.h>
#include <Eigen/Dense>
#include <eigen_stl_containers/eigen_stl_containers.h>
#include <geometry_msgs/PoseArray.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include "tool_path_planner_base.h"


namespace tool_path_planner
{

  /**
   * @brief flipPointOrder Inverts a path, points, normals, and derivatives (not necessarily the spline)
   * @param path The input path to invert
   */
  void flipPointOrder(tool_path_planner::ProcessPath& path);


  std::vector<geometry_msgs::PoseArray> convertVTKtoGeometryMsgs(
      const std::vector<tool_path_planner::ProcessPath>& paths);

  /**
   * @brief conversion function. Not well tested yet
   * @param paths The toolpaths within a raster
   * @return a vector to process paths
   */
  std::vector<tool_path_planner::ProcessPath> toNoetherToolpaths(const noether_msgs::ToolRasterPath& paths);

  /**
   * @brief Convenience conversion function
   * @param paths The tool paths
   * @return  array of PoseArray objects
   */
	std::vector<geometry_msgs::PoseArray> toPosesMsgs(const std::vector<tool_path_planner::ProcessPath>& paths);

	/**
	 * @brief Convenience conversion function.
	 * @param tpp_msg_config  The input tool config message
	 * @return  The Process Tool struct.
	 */
	tool_path_planner::ProcessTool fromTppMsg(const noether_msgs::ToolPathConfig& tpp_msg_config);

	/**
	 * @brief Convenience conversion function
	 * @param tool_config The tool configuration struct
	 * @return  The tool config message
	 */
	noether_msgs::ToolPathConfig toTppMsg(const tool_path_planner::ProcessTool& tool_config);

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

	/**
	 * @details Creates an array of poses from the point cloud with normals
	 * It treats the point cloud as a set of consecutive points and so the x direction is along the line that
	 * connects the current point to the next in the sequence .  The y vector is obtained from the cross product
	 *  of the point normal and the x direction.
	 * @param cloud_normals The points with normals
	 * @param indices       Selects only these points, if left empty all points are used.
	 * @poses               The output pose array
	 * @return  True on success, false otherwise.
	 */
	bool createPoseArray(const pcl::PointCloud<pcl::PointNormal>& cloud_normals, const std::vector<int>& indices,
	                     geometry_msgs::PoseArray& poses);

	template <class C>
	static std::string getClassName() {

	    int status = -4; // some arbitrary value to eliminate the compiler warning
	    const char* mangled_name = typeid(C).name();

	    // enable c++11 by passing the flag -std=c++11 to g++
	    std::unique_ptr<char, void(*)(void*)> res {
	        abi::__cxa_demangle(mangled_name, NULL, NULL, &status),
	        std::free
	    };
	    return (status==0) ? res.get() : mangled_name ;
	}

}



#endif /* INCLUDE_TOOL_PATH_PLANNER_UTILITIES_H_ */
