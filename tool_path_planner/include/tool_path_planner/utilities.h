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
#include <geometry_msgs/PoseArray.h>

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

}



#endif /* INCLUDE_TOOL_PATH_PLANNER_UTILITIES_H_ */
