#include <pcl/common/transforms.h>
#include <noether_tpp/utils.h>

namespace noether
{
Eigen::Vector3d estimateToolPathDirection(const ToolPath& tool_path)
{
  Eigen::Vector3d avg_dir = Eigen::Vector3d::Zero();
  int n = 0;
  for (const ToolPathSegment& seg : tool_path)
  {
    if (seg.size() > 1)
    {
      for (std::size_t i = 0; i < seg.size() - 1; ++i)
      {
        avg_dir += (seg.at(i + 1).translation() - seg.at(i).translation()).normalized();
        ++n;
      }
    }
  }

  if (n < 1)
    throw std::runtime_error("Insufficient number of points to calculate average tool path direction");

  avg_dir /= static_cast<double>(n);

  return avg_dir.normalized();
}

Eigen::Vector3d estimateRasterDirection(const ToolPaths& tool_paths, const Eigen::Vector3d& reference_tool_path_dir)
{
  // First waypoint in the first segment of the first tool path
  const Eigen::Isometry3d first_wp = tool_paths.at(0).at(0).at(0);
  // First waypoint in the first segment of the last tool path
  const Eigen::Isometry3d first_wp_last_path = tool_paths.at(tool_paths.size() - 1).at(0).at(0);

  // Normalize the reference tool path direction
  const Eigen::Vector3d norm_ref_tool_path_dir = reference_tool_path_dir.normalized();

  // Get the vector between the two points
  Eigen::Vector3d raster_dir = first_wp_last_path.translation() - first_wp.translation();

  // Subtract the component of this vector that is parallel to the reference tool path direction
  raster_dir -= raster_dir.dot(norm_ref_tool_path_dir) * norm_ref_tool_path_dir;
  return raster_dir.normalized();
}

std::vector<pcl::PCLPointField>::const_iterator findField(const std::vector<pcl::PCLPointField>& fields,
                                                          const std::string& name)
{
  return std::find_if(
      fields.begin(), fields.end(), [&name](const pcl::PCLPointField& field) { return field.name == name; });
}

std::vector<pcl::PCLPointField>::const_iterator findFieldOrThrow(const std::vector<pcl::PCLPointField>& fields,
                                                                 const std::string& name)
{
  auto it = findField(fields, name);
  if (it == fields.end())
    throw std::runtime_error("Failed to find field '" + name + "'");
  return it;
}

bool hasNormals(const pcl::PolygonMesh& mesh)
{
  auto nx_it = noether::findField(mesh.cloud.fields, "normal_x");
  auto ny_it = noether::findField(mesh.cloud.fields, "normal_y");
  auto nz_it = noether::findField(mesh.cloud.fields, "normal_z");

  return nx_it != mesh.cloud.fields.end() && ny_it != mesh.cloud.fields.end() && nz_it != mesh.cloud.fields.end();
}

Eigen::Vector3f getPoint(const pcl::PCLPointCloud2& cloud, const std::uint32_t pt_idx)
{
  // Find the x, y, and z fields
  auto x_it = noether::findFieldOrThrow(cloud.fields, "x");
  auto y_it = noether::findFieldOrThrow(cloud.fields, "y");
  auto z_it = noether::findFieldOrThrow(cloud.fields, "z");

  // Check that the xyz fields are floats and contiguous
  if ((y_it->offset - x_it->offset != 4) || (z_it->offset - y_it->offset != 4))
    throw std::runtime_error("XYZ fields are not contiguous floats");

  const std::uint32_t offset = pt_idx * cloud.point_step;
  const auto* xyz = reinterpret_cast<const float*>(cloud.data.data() + offset + x_it->offset);
  return Eigen::Map<const Eigen::Vector3f>(xyz);
}

Eigen::Vector3f getFaceNormal(const pcl::PolygonMesh& mesh, const pcl::Vertices& polygon)
{
  if (polygon.vertices.size() < 3)
  {
    std::stringstream ss;
    ss << "Polygon must have at least 3 vertices (" << polygon.vertices.size() << " provided)";
    throw std::runtime_error(ss.str());
  }

  /* Assuming the vertices of the polygon are arranged in clockwise order, the face normal should be the cross product
   * of the vector from vertex 0 to vertex 1 with the vector from vertex 0 to the last vertex. See the examples below
   * for a triangle and quad face.
   *
   *    0--2   0---3
   *    | /    |   |
   *    |/     |   |
   *    1      1---2
   *
   */
  const Eigen::Vector3f pt_0 = getPoint(mesh.cloud, polygon.vertices[0]);
  const Eigen::Vector3f pt_1 = getPoint(mesh.cloud, polygon.vertices[1]);
  const Eigen::Vector3f pt_n = getPoint(mesh.cloud, polygon.vertices.back());

  const Eigen::Vector3f edge_01 = pt_1 - pt_0;
  const Eigen::Vector3f edge_0n = pt_n - pt_0;

  return edge_01.cross(edge_0n).normalized();
}

Eigen::Vector3f getNormal(const pcl::PCLPointCloud2& cloud, const std::uint32_t pt_idx)
{
  auto nx_it = noether::findField(cloud.fields, "normal_x");
  auto ny_it = noether::findField(cloud.fields, "normal_y");
  auto nz_it = noether::findField(cloud.fields, "normal_z");

  if (nx_it == cloud.fields.end() || ny_it == cloud.fields.end() || nz_it == cloud.fields.end())
    throw std::runtime_error("Not all vertex normal fields exist in the point cloud");

  const std::uint32_t offset = pt_idx * cloud.point_step;
  const auto* nx = reinterpret_cast<const float*>(cloud.data.data() + offset + nx_it->offset);
  const auto* ny = reinterpret_cast<const float*>(cloud.data.data() + offset + ny_it->offset);
  const auto* nz = reinterpret_cast<const float*>(cloud.data.data() + offset + nz_it->offset);

  return Eigen::Vector3f(*nx, *ny, *nz);
}

TriangleMesh createTriangleMesh(const pcl::PolygonMesh& input)
{
  TriangleMesh mesh;

  // Add the vertices
  for (std::uint32_t i = 0; i < input.cloud.width * input.cloud.height; ++i)
  {
    mesh.addVertex(i);
  }

  // Add the faces
  for (auto poly = input.polygons.begin(); poly != input.polygons.end(); ++poly)
  {
    if (poly->vertices.size() < 3)
      throw std::runtime_error("Polygon has fewer than 3 vertices");

    // Create individual triangles from the polygon (n_vert - 2 total triangles)
    for (std::size_t tri = 0; tri < poly->vertices.size() - 2; ++tri)
    {
      TriangleMesh::VertexIndices triangle;
      triangle.resize(3);

      // Get the vertices of a triangle with the first point as the common point
      for (std::uint32_t i = 0; i < 3; ++i)
      {
        std::uint32_t v_ind;
        switch (i)
        {
          case 0:
            v_ind = poly->vertices[0];
            break;
          case 1:
            v_ind = poly->vertices[tri + 1];
            break;
          case 2:
            v_ind = poly->vertices[tri + 2];
            break;
        }

        triangle[i] = TriangleMesh::VertexIndex(v_ind);
      }

      mesh.addFace(triangle);
    }
  }

  return mesh;
}

std::tuple<double, std::vector<double>> computeLength(const ToolPathSegment& segment)
{
  double length = 0.0;

  // Set up a container for the distance along the segment of each waypoint
  std::vector<double> dists;
  dists.reserve(segment.size());

  // Add the distance of the first waypoint
  dists.push_back(0.0);

  for (std::size_t i = 0; i < segment.size() - 1; ++i)
  {
    const Eigen::Isometry3d& first = segment.at(i);
    const Eigen::Isometry3d& second = segment.at(i + 1);
    double d = (second.translation() - first.translation()).norm();
    length += d;
    dists.push_back(length);
  }

  return std::make_tuple(length, dists);
}

void printException(const std::exception& e, std::ostream& ss, int level)
{
  ss << std::string(level * 4, ' ') << e.what() << '\n';
  try
  {
    std::rethrow_if_nested(e);
  }
  catch (const std::exception& nested_exception)
  {
    printException(nested_exception, ss, level + 1);
  }
}

pcl::PolygonMesh createPlaneMesh(const float lx, const float ly, const Eigen::Isometry3d& origin)
{
  if (lx <= 0.0)
  {
    throw std::runtime_error("Plane length must be > 0");
  }
  if (ly <= 0.0)
  {
    throw std::runtime_error("Plane width must be > 0");
  }

  pcl::PolygonMesh mesh;

  // Fill in the points of the mesh cloud
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.width = 4;
  cloud.height = 1;
  cloud.is_dense = true;
  cloud.points.reserve(cloud.width);

  pcl::PointXYZ p0 = { -lx / 2, ly / 2, 0 };
  pcl::PointXYZ p1 = { -lx / 2, -ly / 2, 0 };
  pcl::PointXYZ p2 = { lx / 2, -ly / 2, 0 };
  pcl::PointXYZ p3 = { lx / 2, ly / 2, 0 };

  cloud.points.push_back(p0);
  cloud.points.push_back(p1);
  cloud.points.push_back(p2);
  cloud.points.push_back(p3);

  // Transform the pointcloud
  pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
  pcl::transformPointCloud(cloud, transformed_cloud, origin.matrix());
  pcl::toPCLPointCloud2(transformed_cloud, mesh.cloud);

  // Define the polygons of the mesh
  pcl::Vertices polyA;
  polyA.vertices = { 0, 1, 3 };
  pcl::Vertices polyB;
  polyB.vertices = { 1, 2, 3 };

  mesh.polygons.push_back(polyA);
  mesh.polygons.push_back(polyB);

  return mesh;
}

// ----------------------------------------------------------------------------
// -                        Open3D: www.open3d.org                            -
// ----------------------------------------------------------------------------
// Copyright (c) 2018-2024 www.open3d.org
// SPDX-License-Identifier: MIT
// ----------------------------------------------------------------------------
pcl::PolygonMesh createEllipsoidMesh(const float rx,
                                     const float ry,
                                     const float rz,
                                     const int resolution,
                                     const Eigen::Isometry3d& origin)
{
  pcl::PolygonMesh mesh;
  if (rx <= 0.0 || ry <= 0.0 || rz <= 0.0)
  {
    throw std::runtime_error("A semi major axis is <= 0");
  }
  if (resolution <= 2)
  {
    throw std::runtime_error("resolution should be >= 3");
  }

  // Set up cloud of mesh
  pcl::PointCloud<pcl::PointXYZ> cloud;

  // Set up vertices for the poles
  cloud.points.resize(2 * resolution * (resolution - 1 + 2));
  cloud.points[0].x = 0.0;
  cloud.points[0].y = 0.0;
  cloud.points[0].z = rz;
  cloud.points[1].x = 0.0;
  cloud.points[1].y = 0.0;
  cloud.points[1].z = -rz;

  // Set up vertices for the rest of the ellipsoid
  float step = M_PI / (float)resolution;
  for (int i = 1; i < resolution; i++)
  {
    float theta = step * i;
    int base = 2 + 2 * resolution * (i - 1);
    for (int j = 0; j < 2 * resolution; j++)
    {
      float phi = step * j;
      cloud.points[base + j].x = rx * sin(theta) * cos(phi);
      cloud.points[base + j].y = ry * sin(theta) * sin(phi);
      cloud.points[base + j].z = rz * cos(theta);
    }
  }

  // Set up parameters of the cloud
  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;

  // Transform the pointcloud
  pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
  pcl::transformPointCloud(cloud, transformed_cloud, origin.matrix());
  pcl::toPCLPointCloud2(transformed_cloud, mesh.cloud);

  // Triangles for poles.
  for (int j = 0; j < 2 * resolution; j++)
  {
    int j1 = (j + 1) % (2 * resolution);
    int base = 2;
    pcl::Vertices pcl_indices;
    pcl_indices.vertices.emplace_back(0);
    pcl_indices.vertices.emplace_back(base + j);
    pcl_indices.vertices.emplace_back(base + j1);
    mesh.polygons.emplace_back(pcl_indices);

    base = 2 + 2 * resolution * (resolution - 2);
    pcl::Vertices pcl_indices_2;
    pcl_indices_2.vertices.emplace_back(1);
    pcl_indices_2.vertices.emplace_back(base + j1);
    pcl_indices_2.vertices.emplace_back(base + j);
    mesh.polygons.emplace_back(pcl_indices_2);
  }

  // Triangles for non-polar region.
  for (int i = 1; i < resolution - 1; i++)
  {
    int base1 = 2 + 2 * resolution * (i - 1);
    int base2 = base1 + 2 * resolution;
    for (int j = 0; j < 2 * resolution; j++)
    {
      int j1 = (j + 1) % (2 * resolution);
      pcl::Vertices pcl_indices;
      pcl_indices.vertices.emplace_back(base2 + j);
      pcl_indices.vertices.emplace_back(base1 + j1);
      pcl_indices.vertices.emplace_back(base1 + j);
      mesh.polygons.emplace_back(pcl_indices);

      pcl::Vertices pcl_indices_2;
      pcl_indices_2.vertices.emplace_back(base2 + j);
      pcl_indices_2.vertices.emplace_back(base2 + j1);
      pcl_indices_2.vertices.emplace_back(base1 + j1);
      mesh.polygons.emplace_back(pcl_indices_2);
    }
  }

  return mesh;
}

}  // namespace noether
