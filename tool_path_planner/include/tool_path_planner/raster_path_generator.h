/**
 * @file raster_path_generator.h
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

#ifndef INCLUDE_TOOL_PATH_PLANNER_RASTER_PATH_GENERATOR_H_
#define INCLUDE_TOOL_PATH_PLANNER_RASTER_PATH_GENERATOR_H_

#include <geometry_msgs/PoseArray.h>
#include <shape_msgs/Mesh.h>
#include <boost/optional.hpp>

#include "tool_path_planner_base.h"

namespace tool_path_planner {

/**
 * @class tool_path_planner::RasterPathGenerator
 * @details Generates a raster tool path from a surface using the RasterToolPathPlanner and then sequences the path
 *        so that the points are properly ordered relative to the cut direction.
 */
using ToolPaths = std::vector<tool_path_planner::ProcessPath>;

class RasterPathGenerator {
public:
	RasterPathGenerator();
	virtual ~RasterPathGenerator();

	boost::optional< std::vector<ToolPaths> > generate(const tool_path_planner::ProcessTool& path_gen_config,
      const std::vector<pcl::PolygonMesh>& meshes);

  /**
   * @brief plans a tool path using a default configuration
   * @param path_gen_config The configuration data
   * @param mesh  The mesh data on which to generate a tool path
   * @return  The tool path
   */
	boost::optional<std::vector<geometry_msgs::PoseArray>> generate(const tool_path_planner::ProcessTool& path_gen_config,
			const pcl::PolygonMesh& mesh) const;

  /**
   * @brief plans a tool path using a default configuration
   * @param path_gen_config The configuration data
   * @param mesh  The mesh data on which to generate a tool path
   * @return  The tool path
   */
	boost::optional<std::vector<geometry_msgs::PoseArray>> generate(const tool_path_planner::ProcessTool& path_gen_config,
			const shape_msgs::Mesh& mesh) const;

	/**
	 * @brief Generates a ProcessTool configuration structure with default parameters filled in.
	 * @return  The ProcessTool structure.
	 */
  static tool_path_planner::ProcessTool generateDefaultToolConfig();


};

} /* namespace tool_path_planner */

#endif /* INCLUDE_TOOL_PATH_PLANNER_RASTER_PATH_GENERATOR_H_ */
