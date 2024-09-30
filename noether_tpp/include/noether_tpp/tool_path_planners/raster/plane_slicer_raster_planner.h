/**
 * @file plane_slicer_raster_planner.h
 * @copyright Copyright (c) 2021, Southwest Research Institute
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
#pragma once

#include <noether_tpp/tool_path_planners/raster/raster_planner.h>

#include <vtkDoubleArray.h>
#include <vtkPolyData.h>
#include <vtkPoints.h>
#include <vtkSmartPointer.h>

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
 * @brief An implementation of the Raster Planner using a series of parallel cutting planes.
 * This implementation works best on approximately planar parts.
 * The direction generator defines the direction of the raster cut.
 * The cut normal (i.e., the raster step direction) is defined by the cross product of the cut direction and the
 * smallest principal axis of the mesh.
 */
class PlaneSlicerRasterPlanner : public RasterPlanner
{
public:
  PlaneSlicerRasterPlanner(DirectionGenerator::ConstPtr dir_gen, OriginGenerator::ConstPtr origin_gen);

  void setSearchRadius(const double search_radius);
  void setMinSegmentSize(const double min_segment_size);
  void generateRastersBidirectionally(const bool bidirectional);

protected:
  /**
   * @brief Implementation of the tool path planning capability
   * @param mesh
   * @return
   */
  ToolPaths planImpl(const pcl::PolygonMesh& mesh) const;

  /** @brief Flag indicating whether rasters should be generated in the direction of both the cut normal and its
   * negation */
  bool bidirectional_ = true;
  /** @brief Minimum length of valid segment (m) */
  double min_segment_size_;
  /** @brief Search radius for calculating normals (m) */
  double search_radius_;
};

struct PlaneSlicerRasterPlannerFactory : public RasterPlannerFactory
{
  bool bidirectional;
  double min_segment_size;
  double search_radius;

  ToolPathPlanner::ConstPtr create() const override;
};

}  // namespace noether
