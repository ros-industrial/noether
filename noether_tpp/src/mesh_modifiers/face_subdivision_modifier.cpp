#include <noether_tpp/mesh_modifiers/face_subdivision_modifier.h>
#include <noether_tpp/serialization.h>
#include <noether_tpp/utils.h>

#include <algorithm>
#include <cmath>
#include <Eigen/Core>
#include <map>
#include <pcl/common/io.h>
#include <pcl/conversions.h>
#include <pcl/Vertices.h>
#include <queue>

/**
 * @brief Divides a (possibly non-triangular) polygonal mesh face into a set of constituent triangles
 */
std::vector<std::array<pcl::index_t, 3>> polygonToTriangles(const std::vector<pcl::index_t>& poly)
{
  if (poly.size() < 3)
    throw std::runtime_error("Polygon has fewer than 3 vertices");

  const std::size_t n_triangles = poly.size() - 2;
  std::vector<std::array<pcl::index_t, 3>> triangles(n_triangles);

  // Create individual triangles from the polygon (n_vert - 2 total triangles)
  for (std::size_t tri = 0; tri < n_triangles; ++tri)
  {
    std::array<pcl::index_t, 3>& triangle = triangles[tri];

    // Get the vertices of a triangle with the first point as the common point
    for (pcl::index_t i = 0; i < 3; ++i)
    {
      pcl::index_t v_ind;
      switch (i)
      {
        case 0:
          v_ind = poly[0];
          break;
        case 1:
          v_ind = poly[tri + 1];
          break;
        case 2:
          v_ind = poly[tri + 2];
          break;
      }

      triangle[i] = v_ind;
    }
  }

  return triangles;
}

/**
 * @brief Copies a specific vertex to the end of the vertex point cloud
 * @return Index of the copied point
 */
pcl::index_t copyPointToEnd(pcl::PCLPointCloud2& cloud, const pcl::index_t copy_idx)
{
  // Extract the bytes data of the point to copy.
  // Make a copy in the case that cloud.data has no more capacity and a new contiguous block of memory must be allocated
  // for the cloud data, which will invalidate the iterators `start` and `end`
  auto start = cloud.data.begin() + copy_idx * cloud.point_step;
  auto end = start + cloud.point_step;
  std::vector<uint8_t> pt_data(start, end);

  // Insert the point at the end
  std::copy(pt_data.begin(), pt_data.end(), std::back_inserter(cloud.data));

  cloud.width += 1;
  return cloud.width - 1;
}

/**
 * @brief Updates the position (and optionally normal vector and color) of a target vertex by averaging the values of
 * its source vertices
 */
void updatePointData(pcl::PCLPointCloud2& cloud,
                     const pcl::index_t target,
                     const std::initializer_list<pcl::index_t>& sources,
                     const bool has_normals,
                     const bool has_color)
{
  using namespace noether;

  // Overwrite its position with the mean of the source vertices
  {
    Eigen::Vector3f mean = Eigen::Vector3f::Zero();
    for (const pcl::index_t source : sources)
      mean += getPoint(cloud, source);

    getPoint(cloud, target) = mean / sources.size();
  }

  // Overwrite the normal data with the mean of the source vertices
  if (has_normals)
  {
    Eigen::Vector3f mean = Eigen::Vector3f::Zero();
    for (const pcl::index_t source : sources)
      mean += getNormal(cloud, source);

    getNormal(cloud, target) = mean / sources.size();
  }

  // Overwrite the color data with the mean of the source vertices
  if (has_color)
  {
    Eigen::Vector4i mean = Eigen::Vector4i::Zero();
    for (const pcl::index_t source : sources)
      mean += getRgba(cloud, source).cast<int>();

    getRgba(cloud, target) = (mean / sources.size()).cast<uint8_t>();
  }
}

using MeshEdge = std::pair<pcl::index_t, pcl::index_t>;
using MidpointVertexIndexMapping = std::map<MeshEdge, pcl::index_t>;

/**
 * @brief Returns the index in the vertex cloud of the midpoint of an edge.
 * If the midpoint has not been added, it is created and added to the end of the vertex cloud.
 */
pcl::index_t getOrCreateMidpoint(pcl::index_t idx_start,
                                 pcl::index_t idx_end,
                                 pcl::PolygonMesh& mesh,
                                 MidpointVertexIndexMapping& edge_midpoints,
                                 const bool has_normals,
                                 const bool has_color)
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
    const pcl::index_t idx_midpoint = copyPointToEnd(mesh.cloud, idx_start);
    updatePointData(mesh.cloud, idx_midpoint, { idx_start, idx_end }, has_normals, has_color);

    // Add the vertex index to the midpoint map
    edge_midpoints[edge_key] = idx_midpoint;
    return idx_midpoint;
  }
}

/**
 * @brief Applies recursive midpoint subdivision on a single triangle.
 * The newly created midpoints and triangles are added directly to the input mesh
 */
void subdivideTriangleRecursive(const std::array<pcl::index_t, 3>& tri,
                                pcl::PolygonMesh& mesh,
                                MidpointVertexIndexMapping& midpoints,
                                const bool has_normals,
                                const bool has_color,
                                const unsigned recursions)
{
  // Create the edge midpoints
  const pcl::index_t m0 = getOrCreateMidpoint(tri[0], tri[1], mesh, midpoints, has_normals, has_color);
  const pcl::index_t m1 = getOrCreateMidpoint(tri[1], tri[2], mesh, midpoints, has_normals, has_color);
  const pcl::index_t m2 = getOrCreateMidpoint(tri[2], tri[0], mesh, midpoints, has_normals, has_color);

  std::vector<std::array<pcl::index_t, 3>> subdivided_triangles;
  subdivided_triangles.push_back({ tri[0], m0, m2 });
  subdivided_triangles.push_back({ m0, tri[1], m1 });
  subdivided_triangles.push_back({ m2, m1, tri[2] });
  subdivided_triangles.push_back({ m0, m1, m2 });

  if (recursions == 1)
  {
    std::transform(subdivided_triangles.begin(),
                   subdivided_triangles.end(),
                   std::back_inserter(mesh.polygons),
                   [](const std::array<pcl::index_t, 3>& indices) {
                     pcl::Vertices vertex;
                     vertex.vertices.insert(vertex.vertices.end(), indices.begin(), indices.end());
                     return vertex;
                   });
  }
  else
  {
    for (const std::array<pcl::index_t, 3>& subdivided_tri : subdivided_triangles)
    {
      subdivideTriangleRecursive(subdivided_tri, mesh, midpoints, has_normals, has_color, recursions - 1);
    }
  }
}

namespace noether
{
FaceMidpointSubdivisionMeshModifier::FaceMidpointSubdivisionMeshModifier(unsigned n_iterations)
  : MeshModifier(), n_iterations_(n_iterations)
{
}

std::vector<pcl::PolygonMesh> FaceMidpointSubdivisionMeshModifier::modify(const pcl::PolygonMesh& mesh) const
{
  // Create the output mesh
  // Only copy the header and vertex cloud; the polygons will be added separately
  pcl::PolygonMesh output;
  output.header = mesh.header;
  output.cloud = mesh.cloud;

  // Reserve more memory for midpoints and triangles
  const std::size_t n = mesh.cloud.height * mesh.cloud.width;
  const std::size_t n_verts = std::pow(4, n_iterations_) * n;
  const std::size_t n_triangles = std::pow(4, n_iterations_) * mesh.polygons.size();
  output.polygons.reserve(n_triangles);
  output.cloud.data.reserve(n_verts * mesh.cloud.point_step);

  // Flatten the output mesh cloud
  output.cloud.height = 1;
  output.cloud.width = n;
  output.cloud.row_step = 0;

  const bool has_normals = hasNormals(output.cloud);
  const bool has_color = findField(output.cloud.fields, "rgba") != output.cloud.fields.end();

  // Create a mapping of triangle edges to midpoints such that midpoints don't get created on top of one another when
  // seen in different triangles
  MidpointVertexIndexMapping edge_midpoints;

  // Loop over each polygon face
  for (const pcl::Vertices& face : mesh.polygons)
  {
    // Divide the polygon into its constituent triangles
    for (const std::array<pcl::index_t, 3>& tri : polygonToTriangles(face.vertices))
    {
      // Recursively subdivide each triangle of the face
      subdivideTriangleRecursive(tri, output, edge_midpoints, has_normals, has_color, n_iterations_);
    }
  }

  // Update the row step
  output.cloud.row_step = output.cloud.height * output.cloud.width * output.cloud.point_step;

  output.cloud.data.shrink_to_fit();
  output.polygons.shrink_to_fit();

  // Return the modified mesh
  return { output };
}

std::vector<pcl::PolygonMesh> FaceSubdivisionMeshModifier::modify(const pcl::PolygonMesh& mesh) const
{
  // Create the output mesh
  // Only copy the header and vertex cloud; the polygons will be added separately
  pcl::PolygonMesh output;
  output.header = mesh.header;
  output.cloud = mesh.cloud;

  // Reserve memory for added triangles
  output.polygons.reserve(mesh.polygons.size());

  // Flatten the output mesh cloud
  output.cloud.height = 1;
  output.cloud.width = mesh.cloud.height * mesh.cloud.width;
  output.cloud.row_step = 0;

  const bool has_normals = hasNormals(output.cloud);
  const bool has_color = findField(output.cloud.fields, "rgba") != output.cloud.fields.end();

  // Create queue of faces to evaluate for subdivision
  std::queue<std::vector<pcl::index_t>> faces_queue;
  for (const pcl::Vertices& face : mesh.polygons)
    faces_queue.emplace(face.vertices);

  while (!faces_queue.empty())
  {
    std::vector<pcl::index_t> face = faces_queue.front();
    faces_queue.pop();

    if (!requiresSubdivision(output, face))
    {
      // Add face to the output mesh
      pcl::Vertices v;
      v.vertices = face;
      output.polygons.push_back(v);
    }
    else
    {
      // Add a vertex in the middle of each triangle face and subdivide the triangle into 3 sub-triangles
      for (const std::array<pcl::index_t, 3>& tri : polygonToTriangles(face))
      {
        // Copy the first vertex to the end of the cloud and overwrite its position with the mean of the triangle
        // vertices
        const pcl::index_t m = copyPointToEnd(output.cloud, tri.front());
        updatePointData(output.cloud, m, { tri[0], tri[1], tri[2] }, has_normals, has_color);

        // Create new faces and add to stack
        faces_queue.push({ tri[0], tri[1], m });
        faces_queue.push({ m, tri[1], tri[2] });
        faces_queue.push({ tri[0], m, tri[2] });
      }
    }
  }

  // Update the row step
  output.cloud.row_step = output.cloud.height * output.cloud.width * output.cloud.point_step;

  output.cloud.data.shrink_to_fit();
  output.polygons.shrink_to_fit();

  // Return the modified mesh
  return { output };
}

FaceSubdivisionByAreaMeshModifier::FaceSubdivisionByAreaMeshModifier(float max_area)
  : FaceSubdivisionMeshModifier(), max_area_(max_area)
{
}

bool FaceSubdivisionByAreaMeshModifier::requiresSubdivision(const pcl::PolygonMesh& mesh,
                                                            const std::vector<pcl::index_t>& face) const
{
  float area = 0.0f;

  for (const std::array<pcl::index_t, 3>& tri : polygonToTriangles(face))
  {
    Eigen::Map<const Eigen::Vector3f> v0 = getPoint(mesh.cloud, tri[0]);
    Eigen::Map<const Eigen::Vector3f> v1 = getPoint(mesh.cloud, tri[1]);
    Eigen::Map<const Eigen::Vector3f> v2 = getPoint(mesh.cloud, tri[2]);

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
                                                                  const std::vector<pcl::index_t>& face) const
{
  bool requires_subdivision = false;

  for (pcl::index_t i = 0; i < face.size(); ++i)
  {
    // Get the ith and (i+1)th vertices
    // Wrap around (using the modulus) to check the edge from vertex n to vertex 0
    Eigen::Map<const Eigen::Vector3f> v0 = getPoint(mesh.cloud, face[i % face.size()]);
    Eigen::Map<const Eigen::Vector3f> v1 = getPoint(mesh.cloud, face[(i + 1) % face.size()]);

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
Node convert<noether::FaceMidpointSubdivisionMeshModifier>::encode(
    const noether::FaceMidpointSubdivisionMeshModifier& val)
{
  Node node;
  node["n_iterations"] = val.n_iterations_;
  return node;
}

bool convert<noether::FaceMidpointSubdivisionMeshModifier>::decode(const Node& node,
                                                                   noether::FaceMidpointSubdivisionMeshModifier& val)
{
  val.n_iterations_ = YAML::getMember<float>(node, "n_iterations");
  return true;
}

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
