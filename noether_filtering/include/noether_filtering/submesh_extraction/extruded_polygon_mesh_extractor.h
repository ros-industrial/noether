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

#include <noether_filtering/submesh_extraction/submesh_extractor.h>

namespace noether
{
/**
 * @brief Extracts the submesh with the vertices captured inside an extruded polygon defined by the input boundary
 * points
 * @details Operations
 *   - Fit plane to boundary points
 *   - Project boundary points onto fitted plane
 *   - Extrude a prism defined by the projected boundary to the extents of the mesh in the direction of the fitted plane
 * normal
 *   - Identify the vertices within the extruded prism
 *   - Cluster the inlier vertices
 *   - Extract the single cluster of vertices whose centroid is closest to the centroid of the boundary
 *   - Extract the submesh that includes the inlier vertices in the identified cluster
 */
class ExtrudedPolygonSubMeshExtractor : SubMeshExtractor
{
public:
  struct Params
  {
    /** @brief Distance (m) above which boundary points will not be included as inliers to the plane fit */
    double plane_distance_threshold = 1.0;
    /** @brief Minimum number of vertices in a cluster */
    int min_cluster_size = 100;
    /** @brief Maximum number of vertices in a cluster*/
    int max_cluster_size = std::numeric_limits<int>::max();
    /** @brief Minimum distance (m) between clusters */
    double cluster_tolerance = 0.15;
  };

  pcl::PolygonMesh extract(const pcl::PolygonMesh& mesh, const Eigen::MatrixX3d& boundary) const override;

  Params params;
};

}  // namespace noether
