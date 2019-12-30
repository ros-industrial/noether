/**
 * @author Jorge Nicho <jrgnichodevel@gmail.com>
 * @file plane_slicer_raster_generator.h
 * @date Dec 26, 2019
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

#ifndef INCLUDE_PLANE_SLICER_RASTER_GENERATOR_H_
#define INCLUDE_PLANE_SLICER_RASTER_GENERATOR_H_

#include <vtkPoints.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>
#include <boost/optional.hpp>
#include <pcl/PolygonMesh.h>
#include <shape_msgs/Mesh.h>
#include <geometry_msgs/PoseArray.h>
#include <vtkCellLocator.h>
#include <vtkKdTreePointLocator.h>
#include <noether_msgs/ToolRasterPath.h>

namespace tool_path_planner
{
class PlaneSlicerRasterGenerator
{
public:

  struct Config
  {
    double raster_spacing = 0.04;
    double point_spacing = 0.01;
    double raster_rot_offset = 0.0;
    double min_segment_size = 0.01;
    double search_radius = 0.01;
  };

  PlaneSlicerRasterGenerator();
  virtual ~PlaneSlicerRasterGenerator();


  /**
   * @brief sets the input mesh from which raster paths are to be generated
   * @param mesh The mesh input
   */
  void setInput(pcl::PolygonMesh::ConstPtr mesh);

  /**
   * @brief sets the input mesh from which raster paths are to be generated
   * @param mesh The mesh input
   */
  void setInput(const shape_msgs::Mesh& mesh);

  /**
   * @brief Generate the raster paths that follow the contour of the mesh
   * @param config The configuration
   * @return  An array of raster paths or boost::none when it fails.
   */
  boost::optional< std::vector<noether_msgs::ToolRasterPath> > generate(const PlaneSlicerRasterGenerator::Config& config);

  /**
   * @brief Generate the raster paths that follow the contour of the mesh
   * @param mesh  The input mesh from which raster paths will be generated
   * @param config The configuration
   * @return  An array of raster paths or boost::none when it fails.
   */
  boost::optional< std::vector<noether_msgs::ToolRasterPath> > generate(const shape_msgs::Mesh& mesh,
                                                                    const PlaneSlicerRasterGenerator::Config& config);

  /**
   * @brief Generate the raster paths that follow the contour of the mesh
   * @param mesh  The input mesh from which raster paths will be generated
   * @param config The configuration
   * @return  An array of raster paths or boost::none when it fails.
   */
  boost::optional< std::vector<noether_msgs::ToolRasterPath> > generate(pcl::PolygonMesh::ConstPtr mesh,
                                                                    const PlaneSlicerRasterGenerator::Config& config);
  /**
   * @brief the class name
   * @return a string
   */
  std::string getName();

private:

  bool insertNormals(const double search_radius, vtkSmartPointer<vtkPolyData>& data);

  vtkSmartPointer<vtkPolyData> mesh_data_;
  vtkSmartPointer<vtkKdTreePointLocator> kd_tree_;
  vtkSmartPointer<vtkCellLocator> cell_locator_;
};

} /* namespace tool_path_planner */

#endif /* INCLUDE_PLANE_SLICER_RASTER_GENERATOR_H_ */
