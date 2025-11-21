#include <pcl/common/transforms.h>
#include <noether_tpp/utils.h>
#include <algorithm>

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

bool hasNormals(const pcl::PCLPointCloud2& cloud)
{
  auto nx_it = noether::findField(cloud.fields, "normal_x");
  auto ny_it = noether::findField(cloud.fields, "normal_y");
  auto nz_it = noether::findField(cloud.fields, "normal_z");

  return nx_it != cloud.fields.end() && ny_it != cloud.fields.end() && nz_it != cloud.fields.end();
}

Eigen::Map<Eigen::Vector3f> getPoint(pcl::PCLPointCloud2& cloud, const std::uint32_t pt_idx)
{
  // Find the x, y, and z fields
  auto x_it = noether::findFieldOrThrow(cloud.fields, "x");
  auto y_it = noether::findFieldOrThrow(cloud.fields, "y");
  auto z_it = noether::findFieldOrThrow(cloud.fields, "z");

  // Check that the xyz fields are floats and contiguous
  if ((y_it->offset - x_it->offset != 4) || (z_it->offset - y_it->offset != 4))
    throw std::runtime_error("XYZ fields are not contiguous floats");

  const std::uint32_t offset = pt_idx * cloud.point_step;
  auto* xyz = reinterpret_cast<float*>(cloud.data.data() + offset + x_it->offset);
  return Eigen::Map<Eigen::Vector3f>(xyz);
}

Eigen::Map<const Eigen::Vector3f> getPoint(const pcl::PCLPointCloud2& cloud, const std::uint32_t pt_idx)
{
  pcl::PCLPointCloud2& cloud_mutable = const_cast<pcl::PCLPointCloud2&>(cloud);
  Eigen::Map<Eigen::Vector3f> pt_mutable = getPoint(cloud_mutable, pt_idx);
  return Eigen::Map<const Eigen::Vector3f>(pt_mutable.data());
}

Eigen::Map<Eigen::Vector3f> getNormal(pcl::PCLPointCloud2& cloud, const std::uint32_t pt_idx)
{
  auto nx_it = noether::findFieldOrThrow(cloud.fields, "normal_x");
  auto ny_it = noether::findFieldOrThrow(cloud.fields, "normal_y");
  auto nz_it = noether::findFieldOrThrow(cloud.fields, "normal_z");

  // Check that the xyz fields are floats and contiguous
  if ((ny_it->offset - nx_it->offset != 4) || (nz_it->offset - ny_it->offset != 4))
    throw std::runtime_error("Normal fields are not contiguous floats");

  const std::uint32_t offset = pt_idx * cloud.point_step;
  auto* nx = reinterpret_cast<float*>(cloud.data.data() + offset + nx_it->offset);
  return Eigen::Map<Eigen::Vector3f>(nx);
}

Eigen::Map<const Eigen::Vector3f> getNormal(const pcl::PCLPointCloud2& cloud, const std::uint32_t pt_idx)
{
  pcl::PCLPointCloud2& cloud_mutable = const_cast<pcl::PCLPointCloud2&>(cloud);
  Eigen::Map<Eigen::Vector3f> normal_mutable = getNormal(cloud_mutable, pt_idx);
  return Eigen::Map<const Eigen::Vector3f>(normal_mutable.data());
}

Eigen::Map<Eigen::Vector<uint8_t, 4>> getRgba(pcl::PCLPointCloud2& cloud, const std::uint32_t pt_idx)
{
  // Find the rgba field
  auto rgba_it = noether::findFieldOrThrow(cloud.fields, "rgba");

  const std::uint32_t offset = pt_idx * cloud.point_step;
  return Eigen::Map<Eigen::Vector<uint8_t, 4>>(cloud.data.data() + offset + rgba_it->offset);
}

Eigen::Map<const Eigen::Vector<uint8_t, 4>> getRgba(const pcl::PCLPointCloud2& cloud, const std::uint32_t pt_idx)
{
  pcl::PCLPointCloud2& cloud_mutable = const_cast<pcl::PCLPointCloud2&>(cloud);
  Eigen::Map<Eigen::Vector<uint8_t, 4>> rgba_mutable = getRgba(cloud_mutable, pt_idx);
  return Eigen::Map<const Eigen::Vector<uint8_t, 4>>(rgba_mutable.data());
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
  const Eigen::Map<const Eigen::Vector3f> pt_0 = getPoint(mesh.cloud, polygon.vertices[0]);
  const Eigen::Map<const Eigen::Vector3f> pt_1 = getPoint(mesh.cloud, polygon.vertices[1]);
  const Eigen::Map<const Eigen::Vector3f> pt_n = getPoint(mesh.cloud, polygon.vertices.back());

  const Eigen::Vector3f edge_01 = pt_1 - pt_0;
  const Eigen::Vector3f edge_0n = pt_n - pt_0;

  return edge_01.cross(edge_0n).normalized();
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
  if (lx <= 0.0f)
    throw std::runtime_error("Plane length (x) must be > 0");

  if (ly <= 0.0f)
    throw std::runtime_error("Plane length (y) must be > 0");

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

/**
 * Original function from Open3D under MIT license.
 * Copyright (c) 2018-2023 www.open3d.org.
 * Modified by Southwest Research Institute 10/2025.
 */
pcl::PolygonMesh createEllipsoidMesh(const float rx,
                                     const float ry,
                                     const float rz,
                                     const int resolution,
                                     const float theta_range,
                                     const float phi_range,
                                     const Eigen::Isometry3d& origin)
{
  if (rx <= 0 || ry <= 0 || rz <= 0)
    throw std::runtime_error("A semi major axis is <= 0");

  if (resolution < 3)
    throw std::runtime_error("Resolution must be greater than 3");

  // Extract the theta angles
  // The theta angle is the angle that spans from pole to pole; its maximum is 180 degrees
  const float MAX_THETA = static_cast<float>(M_PI);
  Eigen::VectorXf theta(resolution);
  {
    // Extract the appropriate range of theta, bounded by pi
    const float range = std::min(theta_range, MAX_THETA);

    // The points at the poles of the ellipsoid are added separately, so the theta range should start and end a step
    // before the poles The theta angle should be centered on MAX_THETA / 2.0 The desired number of intermediate theta
    // points is `resolution`, so we need `resolution + 1` divisions
    float step = range / (resolution + 1);
    float start = (MAX_THETA - range) / 2.0 + step;
    float end = (MAX_THETA + range) / 2.0 - step;
    theta.setLinSpaced(start, end);
  }

  // Extract the phi angles
  // Phi is the angle around the axis that passes through both poles; its maximum is 360 degrees
  const float MAX_PHI = static_cast<float>(2.0 * M_PI);
  Eigen::VectorXf phi(resolution);
  {
    // Extract the appropriate range of phi, bounded by 360 degrees
    const float range = std::min(phi_range, MAX_PHI);

    // If the full range of phi is used, make `resolution` number of points spaced from zero to one step before
    // `PHI_MAX`
    if (std::abs(range - MAX_PHI) < std::numeric_limits<float>::epsilon())
    {
      float step = MAX_PHI / resolution;
      phi.setLinSpaced(0.0, MAX_PHI - step);
    }
    else
    {
      // If a smaller range of phi is used, center the angle on zero and make sure to start and end at half the range
      float start = -range / 2.0;
      float end = range / 2.0;
      phi.setLinSpaced(start, end);
    }
  }

  // Skip the addition of poles if the theta angle is less than maximum
  const bool skip_poles = theta_range < MAX_THETA;
  const int n_poles = skip_poles ? 0 : 2;

  // Set up a point cloud for the vertices of the mesh
  pcl::PointCloud<pcl::PointXYZ> cloud;
  cloud.points.resize(resolution * resolution + n_poles);

  // Conditionally add the pole vertices as the first two points in the cloud
  if (n_poles > 0)
  {
    // Pole 1
    cloud.points[0].x = 0.0;
    cloud.points[0].y = 0.0;
    cloud.points[0].z = rz;

    // Pole 2
    cloud.points[1].x = 0.0;
    cloud.points[1].y = 0.0;
    cloud.points[1].z = -rz;
  }

  // Add the vertices of the body of the ellipsoid
  for (int i = 0; i < resolution; ++i)
  {
    for (int j = 0; j < resolution; ++j)
    {
      const int idx = n_poles + i * resolution + j;
      cloud.points[idx].x = rx * std::sin(theta(i)) * std::cos(phi(j));
      cloud.points[idx].y = ry * std::sin(theta(i)) * std::sin(phi(j));
      cloud.points[idx].z = rz * std::cos(theta(i));
    }
  }

  // Set up parameters of the cloud
  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;

  // Transform the point cloud
  pcl::PointCloud<pcl::PointXYZ> transformed_cloud;
  pcl::transformPointCloud(cloud, transformed_cloud, origin.matrix());

  pcl::PolygonMesh mesh;
  pcl::toPCLPointCloud2(transformed_cloud, mesh.cloud);

  // Allocate memory for triangles
  const int n_pole_triangles = n_poles * resolution;
  // (# of rings - 1) * (# of squares per ring) * (# of triangles per square)
  const int n_body_triangles = (resolution - 1) * resolution * 2;
  mesh.polygons.reserve(n_pole_triangles + n_body_triangles);

  // Add triangles for the poles
  if (n_poles > 0)
  {
    for (int j = 0; j < resolution; ++j)
    {
      // Index of the first body vertex in the triangle
      const int v1_relative_idx = j;
      // Index of the second body vertex in the triangle (first vertex index plus one)
      // This index wraps around at `resolution + 1` to create a triangle between the pole, the last vertex in the
      // "ring" and the first vertex in the "ring"
      const int v2_relative_idx = (v1_relative_idx + 1) % resolution;

      // Pole 1
      {
        pcl::Vertices tri;
        tri.vertices.emplace_back(0);

        // The index of the first body vertex next to pole 0 is after the pole vertices
        const int start_idx = n_poles;
        tri.vertices.emplace_back(start_idx + v1_relative_idx);
        tri.vertices.emplace_back(start_idx + v2_relative_idx);
        mesh.polygons.emplace_back(tri);
      }

      // Pole 2
      {
        pcl::Vertices tri;
        tri.vertices.emplace_back(1);

        // The index of the first body vertex next to pole 1 is after the pole vertices and after `n-1` "rings" of size
        // `n`
        const int start_idx = n_poles + (resolution - 1) * resolution;
        tri.vertices.emplace_back(start_idx + v2_relative_idx);
        tri.vertices.emplace_back(start_idx + v1_relative_idx);
        mesh.polygons.emplace_back(tri);
      }
    }
  }

  // Triangles for body
  // If the phi angle is not full range, we don't want to connect the last triangles with the first triangles in order
  // to make a shell rather than a closed shape
  const bool skip_connecting_triangles = phi_range < MAX_PHI;
  const int n_phi = skip_connecting_triangles ? resolution - 1 : resolution;

  for (int i = 0; i < resolution - 1; ++i)
  {
    // Theta indices (i.e., "rings")
    int t1_idx = i;
    int t2_idx = i + 1;

    for (int j = 0; j < n_phi; ++j)
    {
      // A quadrilateral is formed between two adjacent vertices on one "ring" and the two adjacent vertices with the
      // same relative index on the next "ring"

      // Vertex at theta ("ring") 1, phi 1
      int t1_p1_idx = n_poles + t1_idx * resolution + j;
      // Vertex at theta ("ring") 1, phi 2
      int t1_p2_idx = n_poles + t1_idx * resolution + ((j + 1) % resolution);
      // Vertex at theta ("ring") 2, phi 1
      int t2_p1_idx = n_poles + t2_idx * resolution + j;
      // Vertex at theta ("ring") 2, phi 2
      int t2_p2_idx = n_poles + t2_idx * resolution + ((j + 1) % resolution);

      // Split this quad into two triangles
      // Triangle 1
      {
        pcl::Vertices tri;
        tri.vertices.emplace_back(t1_p1_idx);
        tri.vertices.emplace_back(t2_p1_idx);
        tri.vertices.emplace_back(t2_p2_idx);
        mesh.polygons.emplace_back(tri);
      }
      // Triangle 2
      {
        pcl::Vertices tri;
        tri.vertices.emplace_back(t1_p1_idx);
        tri.vertices.emplace_back(t2_p2_idx);
        tri.vertices.emplace_back(t1_p2_idx);
        mesh.polygons.emplace_back(tri);
      }
    }
  }

  return mesh;
}

pcl::PolygonMesh createCylinderMesh(const float radius,
                                    const float length,
                                    const int resolution,
                                    const float theta_range,
                                    const bool include_caps,
                                    const Eigen::Isometry3d& origin)
{
  pcl::PointCloud<pcl::PointNormal> cloud;
  cloud.resize(2 * resolution);

  // Add the rings
  // Extract the theta angles
  // Theta is the angle around the axis that passes through both poles; its maximum is 360 degrees
  const float MAX_THETA = static_cast<float>(2.0 * M_PI);
  Eigen::VectorXf theta(resolution);
  {
    // Extract the appropriate range of theta, bounded by 360 degrees
    const float range = std::min(theta_range, MAX_THETA);

    // If the full range of theta is used, make `resolution` number of points spaced from zero to one step before
    // `THETA_MAX`
    if (std::abs(range - MAX_THETA) < std::numeric_limits<float>::epsilon())
    {
      float step = MAX_THETA / resolution;
      theta.setLinSpaced(0.0, MAX_THETA - step);
    }
    else
    {
      // If a smaller range of theta is used, center the angle on zero and make sure to start and end at half the range
      float start = -range / 2.0;
      float end = range / 2.0;
      theta.setLinSpaced(start, end);
    }
  }

  // Add the vertices
  for (int i = 0; i < resolution; ++i)
  {
    // Positive z cap
    {
      pcl::PointNormal& pt = cloud.points[i];
      pt.x = radius * std::cos(theta(i));
      pt.y = radius * std::sin(theta(i));
      pt.z = length / 2.0f;

      pt.normal_x = std::cos(theta[i]);
      pt.normal_y = std::sin(theta[i]);
      pt.normal_z = 0.0;
    }
    // Negative z cap
    {
      pcl::PointNormal& pt = cloud.points[resolution + i];
      pt.x = radius * std::cos(theta(i));
      pt.y = radius * std::sin(theta(i));
      pt.z = -length / 2.0f;

      pt.normal_x = std::cos(theta[i]);
      pt.normal_y = std::sin(theta[i]);
      pt.normal_z = 0.0;
    }
  }

  // Add "pole" points in the center of the "caps"
  if (include_caps)
  {
    {
      pcl::PointNormal pole;
      pole.x = 0.0;
      pole.y = 0.0;
      pole.z = length / 2.0f;
      pole.getNormalVector3fMap() = Eigen::Vector3f::UnitZ();

      cloud.points.push_back(pole);
    }
    {
      pcl::PointNormal pole;
      pole.x = 0.0;
      pole.y = 0.0;
      pole.z = -length / 2.0f;
      pole.getNormalVector3fMap() = -1.0 * Eigen::Vector3f::UnitZ();

      cloud.points.push_back(pole);
    }
  }

  // Set up parameters of the cloud
  cloud.width = cloud.points.size();
  cloud.height = 1;
  cloud.is_dense = true;

  // Transform the point cloud
  pcl::PointCloud<pcl::PointNormal> transformed_cloud;
  pcl::transformPointCloud(cloud, transformed_cloud, origin.matrix());

  pcl::PolygonMesh mesh;
  pcl::toPCLPointCloud2(transformed_cloud, mesh.cloud);

  // Add the triangles for the body of the cylinder
  const bool skip_connecting_triangles = theta_range < MAX_THETA;
  const int n = skip_connecting_triangles ? resolution - 1 : resolution;
  mesh.polygons.reserve(n);
  for (int i = 0; i < n; ++i)
  {
    // A quadrilateral is formed between two adjacent vertices on one cap and the two adjacent vertices with the
    // same relative index on the other cap

    // Vertex at cap 1 (positive), theta 1
    int c1_t1_idx = i;
    // Vertex at cap 1 (positive), theta 2
    int c1_t2_idx = ((c1_t1_idx + 1) % resolution);
    // Vertex at cap 2 (negative), theta 1
    int c2_t1_idx = resolution + i;
    // Vertex at cap 2 (negative), theta 1
    int c2_t2_idx = resolution + ((i + 1) % resolution);

    // Triangle 1
    {
      pcl::Vertices tri;
      tri.vertices.push_back(c1_t1_idx);
      tri.vertices.push_back(c2_t1_idx);
      tri.vertices.push_back(c1_t2_idx);
      mesh.polygons.push_back(tri);
    }

    // Triangle 2
    {
      pcl::Vertices tri;
      tri.vertices.push_back(c2_t1_idx);
      tri.vertices.push_back(c2_t2_idx);
      tri.vertices.push_back(c1_t2_idx);
      mesh.polygons.push_back(tri);
    }
  }

  // Cap triangles
  if (include_caps)
  {
    // Compute the indices of the pole points
    const int idx_pole_1 = resolution * 2;
    const int idx_pole_2 = idx_pole_1 + 1;

    for (int i = 0; i < resolution; ++i)
    {
      // Compute the indices of the two adjacent vertices around the cap
      int idx_v1 = i;
      int idx_v2 = (i + 1) % resolution;

      // Cap for pole 1 (positive end)
      {
        pcl::Vertices tri;
        tri.vertices.push_back(idx_pole_1);
        tri.vertices.push_back(idx_v1);
        tri.vertices.push_back(idx_v2);
        mesh.polygons.push_back(tri);
      }

      // Cap for pole 2 (negative end)
      {
        pcl::Vertices tri;
        tri.vertices.push_back(idx_pole_2);
        // Offset the vertex indices by the number of vertices in the first cap (`resolution`) to get to the start of
        // the second cap Add the triangles in reverse order to get the normal vector facing out
        tri.vertices.push_back(resolution + idx_v2);
        tri.vertices.push_back(resolution + idx_v1);
        mesh.polygons.push_back(tri);
      }
    }
  }

  return mesh;
}

}  // namespace noether
