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

/**
 * @brief Divides a (possibly non-triangular) polygonal mesh face into a set of constituent triangles
 */
std::vector<pcl::Vertices> polygonToTriangles(const pcl::Vertices& poly)
{
  if (poly.vertices.size() < 3)
    throw std::runtime_error("Polygon has fewer than 3 vertices");

  const std::size_t n_triangles = poly.vertices.size() - 2;
  std::vector<pcl::Vertices> triangles(n_triangles);

  // Create individual triangles from the polygon (n_vert - 2 total triangles)
  for (std::size_t tri = 0; tri < n_triangles; ++tri)
  {
    pcl::Vertices& triangle = triangles[tri];
    triangle.vertices.resize(3);

    // Get the vertices of a triangle with the first point as the common point
    for (pcl::index_t i = 0; i < 3; ++i)
    {
      pcl::index_t v_ind;
      switch (i)
      {
        case 0:
          v_ind = poly.vertices[0];
          break;
        case 1:
          v_ind = poly.vertices[tri + 1];
          break;
        case 2:
          v_ind = poly.vertices[tri + 2];
          break;
      }

      triangle.vertices[i] = v_ind;
    }
  }

  return triangles;
}

pcl::Vertices createTriangleVertices(const std::initializer_list<pcl::index_t>& indices)
{
  pcl::Vertices vertex;
  vertex.vertices.insert(vertex.vertices.end(), indices.begin(), indices.end());
  return vertex;
}

using VertexIndex = pcl::index_t;
using MeshEdge = std::pair<VertexIndex, VertexIndex>;
using MidpointVertexIndexMapping = std::map<MeshEdge, VertexIndex>;

pcl::index_t getOrCreateMidpoint(VertexIndex idx_start,
                                 VertexIndex idx_end,
                                 pcl::PolygonMesh& mesh,
                                 MidpointVertexIndexMapping& edge_midpoints,
                                 const bool has_normals,
                                 const bool has_colors)
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
    // Copy the point data from the start waypoint to the end of the cloud data
    {
      const auto* start = mesh.cloud.data.data() + idx_start * mesh.cloud.point_step;
      const auto* end = start + mesh.cloud.point_step;
      mesh.cloud.data.insert(mesh.cloud.data.end(), start, end);
      mesh.cloud.width += 1;
    }

    const pcl::index_t idx_midpoint = static_cast<pcl::index_t>(mesh.cloud.width - 1);

    // Update the position of the midpoint
    {
      Eigen::Map<Eigen::Vector3f> start = noether::getPoint(mesh.cloud, idx_start);
      Eigen::Map<Eigen::Vector3f> end = noether::getPoint(mesh.cloud, idx_end);
      Eigen::Map<Eigen::Vector3f> midpoint = noether::getPoint(mesh.cloud, idx_midpoint);
      midpoint = 0.5f * (start + end);
    }

    // Update the normal
    if (has_normals)
    {
      Eigen::Map<Eigen::Vector3f> start = noether::getNormal(mesh.cloud, idx_start);
      Eigen::Map<Eigen::Vector3f> end = noether::getNormal(mesh.cloud, idx_end);
      Eigen::Map<Eigen::Vector3f> midpoint = noether::getNormal(mesh.cloud, idx_midpoint);
      midpoint = 0.5f * (start + end);
    }

    // Update the color
    if (has_colors)
    {
      Eigen::Map<Eigen::Vector<uint8_t, 4>> start = noether::getRgba(mesh.cloud, idx_start);
      Eigen::Map<Eigen::Vector<uint8_t, 4>> end = noether::getRgba(mesh.cloud, idx_end);
      Eigen::Map<Eigen::Vector<uint8_t, 4>> midpoint = noether::getRgba(mesh.cloud, idx_midpoint);
      midpoint = ((start.cast<unsigned>() + end.cast<unsigned>()) / 2).cast<uint8_t>();
    }

    // Add the vertex index to the midpoint map
    edge_midpoints[edge_key] = idx_midpoint;
    return idx_midpoint;
  }
}

namespace noether
{
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

  const bool has_normals = hasNormals(output.cloud);
  const bool has_color = findField(output.cloud.fields, "rgba") != output.cloud.fields.end();

  // Use a map to store edge midpoints
  MidpointVertexIndexMapping edge_midpoints;

  // Create stack of faces to evaluate for subdivision
  std::queue<pcl::Vertices> faces_queue;
  for (const pcl::Vertices& face : mesh.polygons)
    faces_queue.push(face);

  while (!faces_queue.empty())
  {
    pcl::Vertices face = faces_queue.front();
    faces_queue.pop();

    if (!requiresSubdivision(output, face))
    {
      // Add face to the output mesh
      output.polygons.push_back(face);
    }
    else
    {
      // Subdivide each triangle of the face
      for (const pcl::Vertices& tri : polygonToTriangles(face))
      {
        // Get or create midpoints
        const pcl::index_t m0 =
            getOrCreateMidpoint(tri.vertices[0], tri.vertices[1], output, edge_midpoints, has_normals, has_color);
        const pcl::index_t m1 =
            getOrCreateMidpoint(tri.vertices[1], tri.vertices[2], output, edge_midpoints, has_normals, has_color);
        const pcl::index_t m2 =
            getOrCreateMidpoint(tri.vertices[2], tri.vertices[0], output, edge_midpoints, has_normals, has_color);

        // Create new faces and add to stack
        faces_queue.push(createTriangleVertices({ tri.vertices[0], m0, m2 }));
        faces_queue.push(createTriangleVertices({ m0, tri.vertices[1], m1 }));
        faces_queue.push(createTriangleVertices({ m2, m1, tri.vertices[2] }));
        faces_queue.push(createTriangleVertices({ m0, m1, m2 }));
      }
    }
  }

  // Update the row step
  output.cloud.row_step = output.cloud.height * output.cloud.width * output.cloud.point_step;

  // Return the modified mesh
  return { output };
}

FaceSubdivisionByAreaMeshModifier::FaceSubdivisionByAreaMeshModifier(float max_area)
  : FaceSubdivisionMeshModifier(), max_area_(max_area)
{
}

bool FaceSubdivisionByAreaMeshModifier::requiresSubdivision(const pcl::PolygonMesh& mesh,
                                                            const pcl::Vertices& face) const
{
  float area = 0.0f;

  for (const pcl::Vertices& tri : polygonToTriangles(face))
  {
    Eigen::Map<const Eigen::Vector3f> v0 = getPoint(mesh.cloud, tri.vertices[0]);
    Eigen::Map<const Eigen::Vector3f> v1 = getPoint(mesh.cloud, tri.vertices[1]);
    Eigen::Map<const Eigen::Vector3f> v2 = getPoint(mesh.cloud, tri.vertices[2]);

    float tri_area = 0.5f * ((v1 - v0).cross(v2 - v0)).norm();

    if (!std::isfinite(tri_area))
      throw std::runtime_error("Triangle area is not finite");

    area += tri_area;
  }

  return area > max_area_;
}

FaceSubdivisionByEdgeLengthMeshModifier::FaceSubdivisionByEdgeLengthMeshModifier(float max_edge_length,
                                                                                 float min_edge_length)
  : FaceSubdivisionMeshModifier(), max_edge_length_(max_edge_length), min_edge_length_x2_(min_edge_length * 2.0f)
{
}

bool FaceSubdivisionByEdgeLengthMeshModifier::requiresSubdivision(const pcl::PolygonMesh& mesh,
                                                                  const pcl::Vertices& face) const
{
  bool requires_subdivision = false;

  for (pcl::index_t i = 0; i < face.vertices.size(); ++i)
  {
    // Get the ith and (i+1)th vertices
    // Wrap around (using the modulus) to check the edge from vertex n to vertex 0
    Eigen::Map<const Eigen::Vector3f> v0 = getPoint(mesh.cloud, face.vertices[i % face.vertices.size()]);
    Eigen::Map<const Eigen::Vector3f> v1 = getPoint(mesh.cloud, face.vertices[(i + 1) % face.vertices.size()]);

    const float edge_length = (v1 - v0).norm();

    // If the division of any one edge would be less than the minimimum edge length, do not do the subdivision
    if (edge_length < min_edge_length_x2_)
      return false;

    requires_subdivision |= edge_length > max_edge_length_;
  }

  return requires_subdivision;
}

}  // namespace noether

namespace YAML
{
/** @cond */
Node convert<noether::FaceSubdivisionByAreaMeshModifier>::encode(const noether::FaceSubdivisionByAreaMeshModifier& val)
{
  Node node;
  node["max_area"] = val.max_area_;
  return node;
}

bool convert<noether::FaceSubdivisionByAreaMeshModifier>::decode(const Node& node,
                                                                 noether::FaceSubdivisionByAreaMeshModifier& val)
{
  val.max_area_ = YAML::getMember<float>(node, "max_area");
  return true;
}

Node convert<noether::FaceSubdivisionByEdgeLengthMeshModifier>::encode(
    const noether::FaceSubdivisionByEdgeLengthMeshModifier& val)
{
  Node node;
  node["max_edge_length"] = val.max_edge_length_;
  node["min_edge_length"] = val.min_edge_length_x2_ / 2.0f;
  return node;
}

bool convert<noether::FaceSubdivisionByEdgeLengthMeshModifier>::decode(
    const Node& node,
    noether::FaceSubdivisionByEdgeLengthMeshModifier& val)
{
  val.max_edge_length_ = YAML::getMember<float>(node, "max_edge_length");
  val.min_edge_length_x2_ = YAML::getMember<float>(node, "min_edge_length") * 2.0f;
  return true;
}
/** @endcond */

}  // namespace YAML
