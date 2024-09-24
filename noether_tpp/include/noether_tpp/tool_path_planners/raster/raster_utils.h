#ifndef NOETHER_RASTER_UTILS_H
#define NOETHER_RASTER_UTILS_H

#include <noether_tpp/core/types.h>

#include <vector>
#include <tuple>
#include <Eigen/Core>
#include <Eigen/Geometry>

#include <pcl/PolygonMesh.h>

#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkKdTreePointLocator.h>
#include <vtkDoubleArray.h>

namespace noether
{
// Constants
constexpr double EPSILON = 1e-6;

/**
 * @brief Data structure to hold raster segments and their lengths
 */
struct RasterConstructData
{
  std::vector<vtkSmartPointer<vtkPolyData>> raster_segments;
  std::vector<double> segment_lengths;
};

/**
 * @brief Computes a rotation matrix from the provided orthogonal vectors
 * @param vx X-axis vector
 * @param vy Y-axis vector
 * @param vz Z-axis vector
 * @return Rotation matrix
 */
Eigen::Matrix3d computeRotation(const Eigen::Vector3d& vx, const Eigen::Vector3d& vy, const Eigen::Vector3d& vz);

/**
 * @brief Computes the total length of the provided points
 * @param points VTK points
 * @return Total length
 */
double computeLength(const vtkSmartPointer<vtkPoints>& points);

/**
 * @brief Applies a parametric spline to the provided points and resamples them based on point spacing
 * @param points Input points
 * @param total_length Total length of the points
 * @param point_spacing Desired spacing between points
 * @return New set of points after applying the spline
 */
vtkSmartPointer<vtkPoints> applyParametricSpline(const vtkSmartPointer<vtkPoints>& points,
                                                 double total_length,
                                                 double point_spacing);

/**
 * @brief Removes redundant point indices that appear in multiple lists
 * @param points_lists Vector of point index lists
 */
void removeRedundant(std::vector<std::vector<vtkIdType>>& points_lists);

/**
 * @brief Merges raster segments that are within a certain distance of each other
 * @param points VTK points
 * @param merge_dist Distance threshold for merging
 * @param points_lists Vector of point index lists representing raster segments
 */
void mergeRasterSegments(const vtkSmartPointer<vtkPoints>& points,
                         double merge_dist,
                         std::vector<std::vector<vtkIdType>>& points_lists);

/**
 * @brief Rectifies the direction of raster segments based on a reference point
 * @param points VTK points
 * @param ref_point Reference point for direction rectification
 * @param points_lists Vector of point index lists representing raster segments
 */
void rectifyDirection(const vtkSmartPointer<vtkPoints>& points,
                      const Eigen::Vector3d& ref_point,
                      std::vector<std::vector<vtkIdType>>& points_lists);

/**
 * @brief Converts raster data to tool paths with poses
 * @param rasters_data Vector of raster construction data
 * @return Tool paths generated from the raster data
 */
ToolPaths convertToPoses(const std::vector<RasterConstructData>& rasters_data);

/**
 * @brief Inserts normals into the provided polydata by averaging normals from nearby mesh points
 * @param search_radius Radius to search for nearby points
 * @param mesh_data_ Mesh data containing normals
 * @param kd_tree_ Kd-tree for efficient nearest neighbor search
 * @param data Polydata to insert normals into
 * @return True if normals were successfully inserted, false otherwise
 */
bool insertNormals(const double search_radius,
                   vtkSmartPointer<vtkPolyData>& mesh_data_,
                   vtkSmartPointer<vtkKdTreePointLocator>& kd_tree_,
                   vtkSmartPointer<vtkPolyData>& data);

/**
 * @brief Gets the distances (normal to the cut plane) from the cut origin to the closest and furthest corners of the
 * mesh
 * @param obb_size Column-wise matrix of the object-aligned size vectors of the mesh (from PCA), relative to the mesh
 * origin
 * @param pca_centroid Centroid of the mesh determined by PCA, relative to the mesh origin
 * @param cut_origin Origin of the raster pattern, relative to the mesh origin
 * @param cut_normal Raster cut plane normal, relative to the mesh origin
 * @return Tuple of (min distance, max distance)
 */
std::tuple<double, double> getDistancesToMinMaxCuts(const Eigen::Matrix3d& obb_size,
                                                    const Eigen::Vector3d& pca_centroid,
                                                    const Eigen::Vector3d& cut_origin,
                                                    const Eigen::Vector3d& cut_normal);

/**
 * @brief Converts a pcl::PolygonMesh to a vtkSmartPointer<vtkPolyData> and computes normals if necessary.
 * @param mesh Input PCL PolygonMesh
 * @return vtkSmartPointer<vtkPolyData> containing the converted mesh with normals computed if necessary
 */
vtkSmartPointer<vtkPolyData> convertMeshToVTKPolyData(const pcl::PolygonMesh& mesh);

}  // namespace noether

#endif  // NOETHER_RASTER_UTILS_H
