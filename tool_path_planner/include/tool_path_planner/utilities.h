/*
 *
 * Copyright (c) 2018, Southwest Research Institute
 * All rights reserved.*
 *
 * utilities.h
 *
 *  Created on: Nov 15, 2018
 *      Author: Jorge Nicho
 */


#ifndef INCLUDE_TPP_PATH_GEN_UTILITIES_H_
#define INCLUDE_TPP_PATH_GEN_UTILITIES_H_

#include <noether_msgs/ToolPathConfig.h>
#include <Eigen/Dense>
#include <eigen_stl_containers/eigen_stl_containers.h>
#include <shape_msgs/Mesh.h>
#include <pcl/PolygonMesh.h>
#include <geometry_msgs/PoseArray.h>

#include "tool_path_planner_base.h"


namespace tool_path_planner
{

  /**
   * @brief flipPointOrder Inverts a path, points, normals, and derivatives (not necessarily the spline)
   * @param path The input path to invert
   */
  void flipPointOrder(tool_path_planner::ProcessPath& path);

  /**
   * @brief Convenience conversion function
   * @param paths The tool paths
   * @return  array of PoseArray objects
   */
	std::vector<geometry_msgs::PoseArray> toPosesMsgs(const std::vector<tool_path_planner::ProcessPath>& paths);

	/**
	 * @brief Convenience conversion function
	 * @param mesh_msg  The input mesh
	 * @param mesh      The output pcl mesh
	 * @return  True on success, false otherwise.
	 */
	bool convertToPCLMesh(const shape_msgs::Mesh& mesh_msg, pcl::PolygonMesh& mesh);

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



#endif /* INCLUDE_TPP_PATH_GEN_UTILITIES_H_ */
