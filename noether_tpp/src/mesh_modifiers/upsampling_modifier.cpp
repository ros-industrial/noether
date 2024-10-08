#include <noether_tpp/mesh_modifiers/upsampling_modifier.h>
#include <Eigen/Core>
#include <algorithm>
#include <stack>
#include <stdexcept>

#include <pcl/conversions.h>
#include <pcl/Vertices.h>

pcl::Vertices createTriangleVertices(std::initializer_list<uint32_t> indices)
{
  pcl::Vertices vertex;
  vertex.vertices.insert(vertex.vertices.end(), indices.begin(), indices.end());
  return vertex;
}

namespace noether
{
UpsamplingMeshModifier::UpsamplingMeshModifier(double max_area) : MeshModifier(), max_area_(max_area) {}

std::vector<pcl::PolygonMesh> UpsamplingMeshModifier::modify(const pcl::PolygonMesh& mesh) const
{
  // Convert the input mesh to a point cloud and a list of faces
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::fromPCLPointCloud2(mesh.cloud, *cloud);

  std::vector<pcl::Vertices> faces = mesh.polygons;

  // Copy vertices to a vector
  std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>> vertices = cloud->points;

  // Use a map to store edge midpoints
  std::map<std::pair<uint32_t, uint32_t>, uint32_t> edge_midpoints;

  // List to store final faces
  std::vector<pcl::Vertices> final_faces;

  // Stack to process faces
  std::stack<pcl::Vertices> faces_to_process;
  for (const auto& face : faces)
  {
    faces_to_process.push(face);
  }

  while (!faces_to_process.empty())
  {
    pcl::Vertices face = faces_to_process.top();
    faces_to_process.pop();

    if (face.vertices.size() != 3)
    {
      throw std::runtime_error("Face is not a triangle");
    }

    uint32_t idx0 = face.vertices[0];
    uint32_t idx1 = face.vertices[1];
    uint32_t idx2 = face.vertices[2];

    pcl::PointXYZ v0 = vertices[idx0];
    pcl::PointXYZ v1 = vertices[idx1];
    pcl::PointXYZ v2 = vertices[idx2];

    // Compute area of triangle
    Eigen::Vector3f p0(v0.x, v0.y, v0.z);
    Eigen::Vector3f p1(v1.x, v1.y, v1.z);
    Eigen::Vector3f p2(v2.x, v2.y, v2.z);

    float area = 0.5f * ((p1 - p0).cross(p2 - p0)).norm();

    if (area <= max_area_)
    {
      // Add face to final list
      final_faces.push_back(face);
    }
    else
    {
      // Subdivide the triangle
      // Get or create midpoints
      uint32_t m0 = getOrCreateMidpoint(idx0, idx1, vertices, edge_midpoints);
      uint32_t m1 = getOrCreateMidpoint(idx1, idx2, vertices, edge_midpoints);
      uint32_t m2 = getOrCreateMidpoint(idx2, idx0, vertices, edge_midpoints);

      // Create new faces and add to stack
      faces_to_process.push(createTriangleVertices({ idx0, m0, m2 }));
      faces_to_process.push(createTriangleVertices({ m0, idx1, m1 }));
      faces_to_process.push(createTriangleVertices({ m2, m1, idx2 }));
      faces_to_process.push(createTriangleVertices({ m0, m1, m2 }));
    }
  }

  // Create new mesh
  pcl::PointCloud<pcl::PointXYZ> new_cloud;
  new_cloud.points = vertices;

  pcl::PolygonMesh output_mesh;
  pcl::toPCLPointCloud2(new_cloud, output_mesh.cloud);
  output_mesh.polygons = final_faces;
  output_mesh.header = mesh.header;

  // Return the modified mesh
  return { output_mesh };
}

// Helper function to compute or get the midpoint
uint32_t UpsamplingMeshModifier::getOrCreateMidpoint(
    uint32_t idx_start,
    uint32_t idx_end,
    std::vector<pcl::PointXYZ, Eigen::aligned_allocator<pcl::PointXYZ>>& vertices,
    std::map<std::pair<uint32_t, uint32_t>, uint32_t>& edge_midpoints) const
{
  // Ensure consistent ordering of edge vertices
  std::pair<uint32_t, uint32_t> edge_key = std::minmax(idx_start, idx_end);

  auto it = edge_midpoints.find(edge_key);
  if (it != edge_midpoints.end())
  {
    return it->second;
  }
  else
  {
    pcl::PointXYZ v_start = vertices[idx_start];
    pcl::PointXYZ v_end = vertices[idx_end];
    pcl::PointXYZ midpoint;
    midpoint.x = 0.5f * (v_start.x + v_end.x);
    midpoint.y = 0.5f * (v_start.y + v_end.y);
    midpoint.z = 0.5f * (v_start.z + v_end.z);

    vertices.push_back(midpoint);
    uint32_t idx_midpoint = static_cast<uint32_t>(vertices.size() - 1);
    edge_midpoints[edge_key] = idx_midpoint;
    return idx_midpoint;
  }
}

}  // namespace noether
