/*
 * plane_slicer_raster_generator.h
 *
 *  Created on: Dec 26, 2019
 *      Author: jrgnicho
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
  boost::optional< std::vector<geometry_msgs::PoseArray> > generate(const PlaneSlicerRasterGenerator::Config& config);

  /**
   * @brief Generate the raster paths that follow the contour of the mesh
   * @param mesh  The input mesh from which raster paths will be generated
   * @param config The configuration
   * @return  An array of raster paths or boost::none when it fails.
   */
  boost::optional< std::vector<geometry_msgs::PoseArray> > generate(const shape_msgs::Mesh& mesh,
                                                                    const PlaneSlicerRasterGenerator::Config& config);

  /**
   * @brief Generate the raster paths that follow the contour of the mesh
   * @param mesh  The input mesh from which raster paths will be generated
   * @param config The configuration
   * @return  An array of raster paths or boost::none when it fails.
   */
  boost::optional< std::vector<geometry_msgs::PoseArray> > generate(pcl::PolygonMesh::ConstPtr mesh,
                                                                    const PlaneSlicerRasterGenerator::Config& config);

  std::string getName();

private:

  vtkSmartPointer<vtkPolyData> mesh_data_;
};

} /* namespace tool_path_planner */

#endif /* INCLUDE_PLANE_SLICER_RASTER_GENERATOR_H_ */
