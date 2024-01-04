/*
 * Copyright 2022 Southwest Research Institute
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#pragma once

#include <Eigen/Dense>
#include <pcl/PolygonMesh.h>

namespace noether
{
/**
 * @brief Base class for extracting a point cloud subset from an input given a set of boundary points
 */
struct SubsetExtractor
{
  /**
   * @brief Extracts the inlying subset of points from a cloud given a set of boundary points
   * @param cloud
   * @param boundary - ordered 3D points (row-wise) defining a closed boundary polygon
   * @return
   */
  virtual std::vector<int> extract(const pcl::PCLPointCloud2& cloud, const Eigen::MatrixX3d& boundary) const = 0;
};

/**
 * @brief Base class for extracting a submesh from a mesh given a set of boundary points
 */
struct SubMeshExtractor
{
  /**
   * @brief Extracts a submesh from a mesh given a set of boundary points
   * @param mesh
   * @param boundary - ordered 3D points (row-wise) defining a closed boundary polygon
   * @return
   */
  virtual pcl::PolygonMesh extract(const pcl::PolygonMesh& mesh, const Eigen::MatrixX3d& boundary) const = 0;
};

/**
 * @brief Extracts the sub-mesh from inlier vertices
 */
pcl::PolygonMesh extractSubMeshFromInlierVertices(const pcl::PolygonMesh& input_mesh,
                                                  const std::vector<int>& inlier_vertex_indices);

}  // namespace noether
