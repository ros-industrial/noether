/**
 * @file mesh_segmenter.h
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

#ifndef MESH_SEGMENTER_H
#define MESH_SEGMENTER_H

#include <vtkPolyData.h>
#include <vtkTriangleFilter.h>
#include <vtkSmartPointer.h>

namespace mesh_segmenter
{
class MeshSegmenter
{
public:
  /**
   * @brief Sets the minimum cluster size - the smallest number of cells allowed in a segment
   * @param x The size to which the paramter is set
   */
  inline void setMinClusterSize(int x) { min_cluster_size_ = x; }
  /**
   * @brief Sets the maximum cluster size - the largest number of cells allowed in a segment - Currently this limitation
   * is unimplemented
   * @param x The size to which the parameter is set
   */
  inline void setMaxClusterSize(int x) { max_cluster_size_ = x; }
  /**
   * @brief Sets the curvature threshold used to divide the segments.
   * @param x The desired threshold for "nearness" in radians, 1 - must be perfectly aligned, 0 - any angle is accepted
   */
  inline void setCurvatureThreshold(double x) { curvature_threshold_ = x; }
  /**
   * @brief Returns the minimum cluster size - the smallest number of cells allowed in a segment
   * @return The minimum cluster size
   */
  inline int getMinClusterSize() { return min_cluster_size_; }
  /**
   * @brief Returns the maximum cluster size - the largest number of cells allowed in a segment - Currently this
   * limitation is unimplemented
   * @return The maximimum cluster size
   */
  inline int getMaxClusterSize() { return max_cluster_size_; }
  /**
   * @brief Returns the curvature threshold used to divide the segments.
   * @return The desired threshold for "nearness" in radians, 1 - must be perfectly aligned, 0 - any angle is accepted
   */
  inline double getCurvatureThreshold() { return curvature_threshold_; }

  /**
   * @brief empty constructor that sets default values of min_cluster_size_ = 50, max_cluster_size_ = 1000000, and
   * curvature_threshold_ = 0.3
   */
  MeshSegmenter() : min_cluster_size_(50), max_cluster_size_(1000000), curvature_threshold_(0.3)
  {
    vtkObject::GlobalWarningDisplayOff();
  }

  /**
   * @brief setInputMesh Set the input mesh to be segmented
   * @param mesh The input mesh to operate on
   */
  void setInputMesh(vtkSmartPointer<vtkPolyData> mesh);

  /**
   * @brief getInputMesh Get the input_mesh_
   * @return The input_mesh_
   */
  vtkSmartPointer<vtkPolyData> getInputMesh() { return input_mesh_; }

  /**
   * @brief getNeighborCells Given a mesh and a target cell, find all other connecting cells
   * @param mesh The input mesh to operate on
   * @param cellId The cell id to find adjacent cells
   * @return The list of cell ids which are adjacent to the target cell
   */
  vtkSmartPointer<vtkIdList> getNeighborCells(int cell_id);

  /**
   * @brief segmentMesh Segments the input mesh using the desired segmentation algorithm
   */
  void segmentMesh();

  /**
   * @brief getMeshSegments Get the segments of the mesh after segmentation has been performed
   * @return The vector of mesh segments
   */
  std::vector<vtkSmartPointer<vtkPolyData> > getMeshSegments();

  /**
   * @brief segmentMesh Performs segmentation on the input mesh, starting at the start_cell id
   * @param start_cell The id of the cell to start segmentation at
   * @return The list of ids associated with the final segmentation
   */
  vtkSmartPointer<vtkIdList> segmentMesh(int start_cell);

  /**
   * @brief areNormalsNear Checks to see if two normal vectors are near each other (within a given threshold)
   * @param norm1 The first normal to check
   * @param norm2 The second normal to check
   * @param threshold The desired threshold for "nearness" in radians, 1 - must be perfectly aligned, 0 - any angle is
   * accepted
   * @return True if the normals are near, False if they are not
   */
  bool areNormalsNear(const double* norm1, const double* norm2, const double threshold);

private:
  /** @brief the smallest number of cells allowed in a segment*/
  int min_cluster_size_;
  /** @brief Currently unimplemented 12/31/2018 - should be the largest number of cells allowed in a segment */
  int max_cluster_size_;
  /** @brief The desired threshold for "nearness" in radians
   */
  double curvature_threshold_;

  /** @brief The input mesh to segment */
  vtkSmartPointer<vtkPolyData> input_mesh_;
  /** @brief VTK triangle filter for finding adjacent cells */
  vtkSmartPointer<vtkTriangleFilter> triangle_filter_;
  /** @brief A list of all indices which indicates which cells belong to which segmentation chuncks */
  std::vector<vtkSmartPointer<vtkIdList> > included_indices_;
};
}  // namespace mesh_segmenter

#endif  // MESH_SEGMENTER_H
