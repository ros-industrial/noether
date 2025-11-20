#include <noether_tpp/mesh_modifiers/face_subdivision_modifier.h>
#include <noether_tpp/serialization.h>
#include <noether_tpp/utils.h>

#include <algorithm>
#include <Eigen/Core>
#include <map>
#include <pcl/common/io.h>
#include <pcl/conversions.h>
#include <pcl/Vertices.h>
#include <queue>
#include <stdexcept>

pcl::Vertices createTriangleVertices(const std::initializer_list<uint32_t>& indices)
{
  pcl::Vertices vertex;
  vertex.vertices.insert(vertex.vertices.end(), indices.begin(), indices.end());
  return vertex;
}

using VertexIndex = uint32_t;
using MeshEdge = std::pair<VertexIndex, VertexIndex>;
using MidpointVertexIndexMapping = std::map<MeshEdge, VertexIndex>;

uint32_t getOrCreateMidpoint(VertexIndex idx_start,
                             VertexIndex idx_end,
                             pcl::PolygonMesh& mesh,
                             MidpointVertexIndexMapping& edge_midpoints)
{
  // Ensure consistent ordering of edge vertices
  MeshEdge edge_key = std::minmax(idx_start, idx_end);

  auto it = edge_midpoints.find(edge_key);
  if (it != edge_midpoints.end())
  {
    return it->second;
  }
  else
  {
    Eigen::Map<Eigen::Vector3f> v_start = noether::getPoint(mesh.cloud, idx_start);
    Eigen::Map<Eigen::Vector3f> v_end = noether::getPoint(mesh.cloud, idx_end);

    // Copy v_start as midpoint
    {
      const auto* start = mesh.cloud.data.data() + idx_start * mesh.cloud.point_step;
      const auto* end = start + mesh.cloud.point_step;
      mesh.cloud.data.insert(mesh.cloud.data.end(), start, end);
      mesh.cloud.width += 1;
    }

    const uint32_t idx_midpoint = static_cast<uint32_t>(mesh.cloud.width - 1);

    // Update the position of the midpoint
    Eigen::Map<Eigen::Vector3f> midpoint = noether::getPoint(mesh.cloud, idx_midpoint);
    midpoint = 0.5f * (v_start + v_end);

    // TODO: Update the normal
    // TODO: Update the color

    // Add the vertex index to the midpoint map
    edge_midpoints[edge_key] = idx_midpoint;
    return idx_midpoint;
  }
}

namespace noether
{
FaceSubdivisionMeshModifier::FaceSubdivisionMeshModifier(double max_area) : MeshModifier(), max_area_(max_area) {}

std::vector<pcl::PolygonMesh> FaceSubdivisionMeshModifier::modify(const pcl::PolygonMesh& mesh) const
{
  pcl::PolygonMesh output;
  output.header = mesh.header;

  // Copy the vertex cloud from the original mesh
  output.cloud = mesh.cloud;

  {
    // Reserve more memory for midpoints
    const std::size_t n = mesh.cloud.height * mesh.cloud.width;
    output.cloud.data.reserve(2 * n * mesh.cloud.point_step);

    // Flatten the output mesh cloud
    output.cloud.height = 1;
    output.cloud.width = n;
  }

  // Use a map to store edge midpoints
  MidpointVertexIndexMapping edge_midpoints;

  // Stack to process faces
  std::queue<pcl::Vertices> faces_queue;
  for (const pcl::Vertices& face : mesh.polygons)
    faces_queue.push(face);

  while (!faces_queue.empty())
  {
    pcl::Vertices face = faces_queue.front();
    faces_queue.pop();

    if (face.vertices.size() != 3)
    {
      throw std::runtime_error("Face is not a triangle");
    }

    uint32_t idx0 = face.vertices[0];
    uint32_t idx1 = face.vertices[1];
    uint32_t idx2 = face.vertices[2];

    Eigen::Map<Eigen::Vector3f> v0 = getPoint(output.cloud, idx0);
    Eigen::Map<Eigen::Vector3f> v1 = getPoint(output.cloud, idx1);
    Eigen::Map<Eigen::Vector3f> v2 = getPoint(output.cloud, idx2);

    if (v0.isApprox(v1))
      throw std::runtime_error("You've done something wrong");

    if (v0.isApprox(v2))
      throw std::runtime_error("You've done something wrong again");

    float area = 0.5f * ((v1 - v0).cross(v2 - v0)).norm();

    if (!std::isfinite(area))
      throw std::runtime_error("Invalid area");

    if (area <= max_area_)
    {
      // Add face to final list
      output.polygons.push_back(face);
    }
    else
    {
      // Subdivide the triangle
      // Get or create midpoints
      uint32_t m0 = getOrCreateMidpoint(idx0, idx1, output, edge_midpoints);
      uint32_t m1 = getOrCreateMidpoint(idx1, idx2, output, edge_midpoints);
      uint32_t m2 = getOrCreateMidpoint(idx2, idx0, output, edge_midpoints);

      // Create new faces and add to stack
      faces_queue.push(createTriangleVertices({ idx0, m0, m2 }));
      faces_queue.push(createTriangleVertices({ m0, idx1, m1 }));
      faces_queue.push(createTriangleVertices({ m2, m1, idx2 }));
      faces_queue.push(createTriangleVertices({ m0, m1, m2 }));
    }
  }

  // Return the modified mesh
  return { output };
}

}  // namespace noether

namespace YAML
{
/** @cond */
Node convert<noether::FaceSubdivisionMeshModifier>::encode(const noether::FaceSubdivisionMeshModifier& val)
{
  Node node;
  node["max_area"] = val.max_area_;
  return node;
}

bool convert<noether::FaceSubdivisionMeshModifier>::decode(const Node& node,
                                                              noether::FaceSubdivisionMeshModifier& val)
{
  val.max_area_ = YAML::getMember<double>(node, "max_area");
  return true;
}
/** @endcond */

}  // namespace YAML
