/*
 * Copyright (c) 2016, Southwest Research Institute
 * All rights reserved.
 *
 */


#ifndef NOETHER_VTK_UTILS_H
#define NOETHER_VTK_UTILS_H

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
   * @brief sampleMesh Uniformly samples point on a mesh
   * @param mesh The input mesh to sample
   * @param distance The spacing between sample points on the surface
   * @return The set of points located on the surface of the mesh
   */
  vtkSmartPointer<vtkPolyData> sampleMesh(vtkSmartPointer<vtkPolyData> mesh, double distance);

  /**
   * @brief loadPCDFile Load a PCL PCD file and convert it to VTK
   * @param file The input file to load
   * @param polydata [output] The VTK object to return
   * @param background Optional point cloud to be used to perform background subtraction
   * @param return_mesh If true, computes and returns a mesh, if false, will only return the point data
   * @return True if the file exists and was loaded, False if there was an error
   */
  bool loadPCDFile(std::string file, vtkSmartPointer<vtkPolyData>& polydata, std::string background = "", bool return_mesh = true);

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

  /**
   * @brief cutMesh Given a list of points, will cut a section out of mesh
   * @param mesh The mesh to cut from
   * @param points The list of points defining a loop in 3D space
   * @param get_inside What to return.  If true, will return the inside section, if false, will return the outside section
   * @return The resulting mesh after cutting
   */
  vtkSmartPointer<vtkPolyData> cutMesh(vtkSmartPointer<vtkPolyData>& mesh, vtkSmartPointer<vtkPoints>& points, bool get_inside);

  /**
   * @brief pclEstimateNormals Wrapper around PCL's normal estimation.  Estimates normals and appends them to the input cloud
   * @param cloud The input cloud without normals
   * @param radius The radius to use for the nearest neighbors search for estimating normals
   * @param view_point The point of view of the 'observer' for the purposes of determining the orientation of the normals
   * @return The output cloud with normals
   */
  pcl::PointCloud<pcl::PointNormal>::Ptr pclEstimateNormals(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud, double radius = 0.01,
                                                            const pcl::PointXYZ& view_point = pcl::PointXYZ(0, 0, 5.0));

  /**
   * @brief pclGridProjectionMesh Wrapper around PCL's grid projection meshing algorithm
   * @param cloud
   * @param resolution
   * @param padding_size
   * @param max_binary_searc_level
   * @param nearest_neighbors
   * @return
   */
  pcl::PolygonMesh pclGridProjectionMesh(pcl::PointCloud<pcl::PointNormal>::ConstPtr cloud, double resolution = 0.003,
                                         int padding_size = 1, int max_binary_searc_level = 6,
                                         int nearest_neighbors = 20);

  /**
   * @brief pclEncodeMeshAndNormals Computes normals for a mesh and converts it to a VTK polydata type (useful for path planning)
   * @param pcl_mesh The input PCL mesh object, without normals
   * @param vtk_mesh The output VTK mesh object, with normals
   * @param radius The radius to use for estimating normals
   * @param view_point The point of view of the 'observer' for the purposes of determining the orientation of the normals
   */
  void pclEncodeMeshAndNormals(const pcl::PolygonMesh& pcl_mesh, vtkSmartPointer<vtkPolyData>& vtk_mesh, double radius = 0.01,
                               const pcl::PointXYZ& view_point = pcl::PointXYZ(0, 0, 5.0));

  /**
   * @brief loadPolygonMeshFromPLY Load a pcl::PolygonMesh from a file
   * @param file The file to read from
   * @param mesh The mesh to return
   * @return True if the file exists and was loaded correctly, false if there was a failure
   */
  bool loadPolygonMeshFromPLY(const std::string& file, pcl::PolygonMesh& mesh);


}
#endif // VTK_UTILS_H
