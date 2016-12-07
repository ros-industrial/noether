/*
 * Copyright (c) 2016, Southwest Research Institute
 * All rights reserved.
 *
 */


#ifndef VTK_UTILS_H
#define VTK_UTILS_H

#include <pcl/common/common.h>
#include <pcl/point_traits.h>
#include <pcl/point_types.h>

#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>

namespace vtk_viewer
{

  /**
   * @brief createPlane Creates a surface mesh that has a sinusoidal function
   * @return The points associated with the mesh
   */
  vtkSmartPointer<vtkPoints> createPlane();

  /**
   * @brief visualizePlane Creates a simple VTK viewer to visualize a mesh object
   * @param polydata The input mesh object to visualize
   */
  void visualizePlane(vtkSmartPointer<vtkPolyData>& polydata);

  /**
   * @brief createMesh Creates a Delaunay2D triangulated mesh from a set of points
   * @param points The set of input points
   * @return The mesh object after triangulation
   */
  vtkSmartPointer<vtkPolyData> createMesh(vtkSmartPointer<vtkPoints> points);

  vtkSmartPointer<vtkPolyData> readSTLFile(std::string file);

  vtkSmartPointer<vtkPolyData> estimateCurvature(vtkSmartPointer<vtkPolyData> mesh, int method);

  void generateNormals(vtkSmartPointer<vtkPolyData>& data);

  vtkSmartPointer<vtkPolyData> upsampleMesh(vtkSmartPointer<vtkPolyData> mesh, double distance);

  bool loadPCDFile(std::string file, vtkSmartPointer<vtkPolyData>& polydata);

  void PCLtoVTK(const pcl::PointCloud<pcl::PointXYZ>& cloud, vtkPolyData* const pdata);

}
#endif // VTK_UTILS_H
