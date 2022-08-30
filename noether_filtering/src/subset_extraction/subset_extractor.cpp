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
#include <noether_filtering/subset_extraction/subset_extractor.h>

#include <pcl/surface/simplification_remove_unused_vertices.h>

namespace noether
{
pcl::PolygonMesh extractSubMeshFromInlierVertices(const pcl::PolygonMesh& input_mesh,
                                                  const std::vector<int>& inlier_vertex_indices)
{
  // mark inlying points as true and outlying points as false
  std::vector<bool> whitelist(input_mesh.cloud.width * input_mesh.cloud.height, false);
  for (std::size_t i = 0; i < inlier_vertex_indices.size(); ++i)
  {
    whitelist[static_cast<std::size_t>(inlier_vertex_indices[i])] = true;
  }

  // All points marked 'false' in the whitelist will be removed. If
  // all of the polygon's points are marked 'true', that polygon
  // should be included.
  pcl::PolygonMesh intermediate_mesh;
  intermediate_mesh.cloud = input_mesh.cloud;
  for (std::size_t i = 0; i < input_mesh.polygons.size(); ++i)
  {
    if (whitelist[input_mesh.polygons[i].vertices[0]] && whitelist[input_mesh.polygons[i].vertices[1]] &&
        whitelist[input_mesh.polygons[i].vertices[2]])
    {
      intermediate_mesh.polygons.push_back(input_mesh.polygons[i]);
    }
  }

  // Remove unused points and save the result to the output mesh
  pcl::surface::SimplificationRemoveUnusedVertices simplifier;
  pcl::PolygonMesh output_mesh;
  simplifier.simplify(intermediate_mesh, output_mesh);

  return output_mesh;
}

}  // namespace noether
