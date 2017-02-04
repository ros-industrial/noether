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
#include <pcl/PolygonMesh.h>

#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkCutter.h>

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
   * @brief createMesh Uses surface reconstruction to approximate a mesh to fit the data
   * @param points The set of input points
   * @param sample_spacing The size of the triangles to form in the final mesh
   * @param neighborhood_size The number of points to consider for determining cell size and orientation
   * @return The mesh object after triangulation
   */
  vtkSmartPointer<vtkPolyData> createMesh(vtkSmartPointer<vtkPoints> points, double sample_spacing, double neigborhood_size);

  /**
   * @brief cleanMesh Given a mesh, removes cells that do not have enough point data nearby (called after createMesh)
   * @param points The set of input points used to eliminate empty/extraneous cells
   * @param mesh The input mesh to filter and remove empty cells from
   */
  void cleanMesh(const vtkSmartPointer<vtkPoints>& points, vtkSmartPointer<vtkPolyData>& mesh);

  /**
   * @brief readSTLFile Reads an STL file and returns a VTK data object
   * @param file The input file to read
   * @return A VTK polydata object representing the CAD file, returns a null pointer if the operation failed
   */
  vtkSmartPointer<vtkPolyData> readSTLFile(std::string file);

  /**
   * @brief estimateCurvature Estimates the curvature of a given mesh (currently experimental)
   * @param mesh The input mesh to operate on
   * @param method The desired method for curvature estimation [0-3]
   * @return
   */
  vtkSmartPointer<vtkPolyData> estimateCurvature(vtkSmartPointer<vtkPolyData> mesh, int method);

  /**
   * @brief generateNormals Generate point and cell (surface) normals (in place)
   * @param data The mesh to generate and add normals to
   */
  void generateNormals(vtkSmartPointer<vtkPolyData>& data, int flip_normals = 1);

  /**
   * @brief upsampleMesh Uniformly samples a mesh to generate a denser mesh (currently experimental and not working)
   * @param mesh The input mesh to sample
   * @param distance The spacing between sample points on the surface
   * @return A new upsampled mesh
   */
  vtkSmartPointer<vtkPolyData> upsampleMesh(vtkSmartPointer<vtkPolyData> mesh, double distance);

  /**
   * @brief loadPCDFile Load a PCL PCD file and convert it to VTK
   * @param file The input file to load
   * @param polydata [output] The VTK object to return
   * @param background Optional point cloud to be used to perform background subtraction
   * @param return_mesh If true, computes and returns a mesh, if false, will only return the point data
   * @return True if the file exists and was loaded, False if there was an error
   */
  bool loadPCDFile(std::string file, vtkSmartPointer<vtkPolyData>& polydata, std::string background = "", bool return_mesh = true);

  void pclGridProjectionMesh(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, vtkSmartPointer<vtkPolyData>& mesh, pcl::PolygonMesh &pcl_mesh);

  pcl::PolygonMesh pclGridProjectionMesh(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

  void vtkSurfaceReconstructionMesh(const pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, vtkSmartPointer<vtkPolyData>& mesh);

  /**
   * @brief PCLtoVTK Converts a PCL point cloud to VTK format
   * @param cloud The input PCL point cloud
   * @param pdata the output VTK data object
   */
  void PCLtoVTK(const pcl::PointCloud<pcl::PointXYZ>& cloud, vtkPolyData* const pdata);

  void VTKtoPCL(vtkPolyData* const pdata, pcl::PointCloud<pcl::PointNormal> &cloud);

  /**
   * @brief removeBackground Removes points from an input cloud using a background cloud as a reference (also removes NaN's)
   * @param cloud The input cloud to perform background subtraction on
   * @param background The reference background cloud
   */
  void removeBackground(pcl::PointCloud<pcl::PointXYZ>& cloud, const pcl::PointCloud<pcl::PointXYZ>& background);

  /**
   * @brief pt_dist Computes the sum of the square distance between two points (no sqrt() to save time)
   * @param pt1 Pointer to a double array of size 3, first point for calculating distance
   * @param pt2 Pointer to a double array of size 3, second point for calculating distance
   * @return The calculated sum squared distance
   */
  double pt_dist(double* pt1, double* pt2);

  vtkSmartPointer<vtkPolyData> cutMesh(vtkSmartPointer<vtkPolyData>& mesh, vtkSmartPointer<vtkPoints>& points, bool get_inside);

}
#endif // VTK_UTILS_H
