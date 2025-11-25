#include <noether_tpp/core/tool_path_modifier.h>

#include <pcl/PCLPointField.h>
#include <pcl/geometry/mesh_traits.h>
#include <pcl/geometry/triangle_mesh.h>
#include <pcl/PolygonMesh.h>

namespace noether
{
/** @brief Estimates the direction of travel of a tool path by averaging the vectors between adjacent waypoints */
Eigen::Vector3d estimateToolPathDirection(const ToolPath& tool_path);

/**
 * @brief Estimates the raster direction of a set of tool paths by returning the component of the vector from the first
 * point in the first segment of the first tool path) to the first waypoint in the first segment of the last tool path
 * that is perpendicular to the nominal tool path direction
 */
Eigen::Vector3d estimateRasterDirection(const ToolPaths& tool_paths, const Eigen::Vector3d& reference_tool_path_dir);

std::vector<pcl::PCLPointField>::const_iterator findField(const std::vector<pcl::PCLPointField>& fields,
                                                          const std::string& name);

std::vector<pcl::PCLPointField>::const_iterator findFieldOrThrow(const std::vector<pcl::PCLPointField>& fields,
                                                                 const std::string& name);

bool hasNormals(const pcl::PCLPointCloud2& cloud);

Eigen::Map<const Eigen::Vector3f> getPoint(const pcl::PCLPointCloud2& cloud, const std::uint32_t pt_idx);
Eigen::Map<Eigen::Vector3f> getPoint(pcl::PCLPointCloud2& cloud, const std::uint32_t pt_idx);

Eigen::Map<const Eigen::Vector3f> getNormal(const pcl::PCLPointCloud2& cloud, const std::uint32_t pt_idx);
Eigen::Map<Eigen::Vector3f> getNormal(pcl::PCLPointCloud2& cloud, const std::uint32_t pt_idx);

Eigen::Map<const Eigen::Vector<uint8_t, 4>> getRgba(const pcl::PCLPointCloud2& cloud, const std::uint32_t pt_idx);
Eigen::Map<Eigen::Vector<uint8_t, 4>> getRgba(pcl::PCLPointCloud2& cloud, const std::uint32_t pt_idx);

Eigen::Vector3f getFaceNormal(const pcl::PolygonMesh& mesh, const pcl::Vertices& polygon);

using MeshTraits = pcl::geometry::DefaultMeshTraits<std::uint32_t>;
using TriangleMesh = pcl::geometry::TriangleMesh<MeshTraits>;

/**
 * @brief Creates a triangle mesh representation of the input polygon mesh
 */
TriangleMesh createTriangleMesh(const pcl::PolygonMesh& input);

/**
 * @brief Computes the length of a tool path segment
 */
std::tuple<double, std::vector<double>> computeLength(const ToolPathSegment& segment);

/**
 * @brief Unrolls an exception and captures content from all nested exceptions
 * @details Adapted from https://en.cppreference.com/w/cpp/error/throw_with_nested
 */
void printException(const std::exception& e, std::ostream& ss, int level = 0);

/**
 * @brief Creates a mesh of an x-y plane (z-axis is normal) with a specified length and width.
 * @details The plane is comprised of 4 vertices and two triangles per the diagram below:
 *
 * 0 --- 3 \n
 * | A / | \n
 * | / B | \n
 * 1 --- 2 \n
 *
 * @param lx Length (m) along the x direction
 * @param ly Length (m) along the y direction
 * @param origin Transform to the desired origin of the primitive
 */
pcl::PolygonMesh createPlaneMesh(const float lx = 1.0,
                                 const float ly = 1.0,
                                 const Eigen::Isometry3d& origin = Eigen::Isometry3d::Identity());

/**
 * @details Creates a mesh of an ellipsoid
 * @sa The implementation is adapted from <a
 * href="https://github.com/isl-org/Open3D/blob/v0.19.0/cpp/open3d/geometry/TriangleMeshFactory.cpp#L216-L381">Open3D</a>
 * @param rx Radius (m) along the x direction
 * @param ry Radius (m) along the y direction
 * @param rz Radius (m) along the z direction
 * @param resolution Number of vertices in each "ring" of the ellipsoid
 * @param theta_range Angle range (radians) of the ellipsoid spanning between the two poles on (0, pi]. If the value is
 * less than pi, an ellipsoid shell is created.
 * @param phi_range Angle range (radians) around the z-axis that passes through both poles, on (0, 2 * pi]. If the value
 * is less than 2 * pi, an ellipsoid shell is crated.
 * @param origin Transform to the desired origin of the primitive
 */
pcl::PolygonMesh createEllipsoidMesh(const float rx = 1.0,
                                     const float ry = 1.0,
                                     const float rz = 1.5,
                                     const std::size_t resolution = 20lu,
                                     const float theta_range = static_cast<float>(M_PI),
                                     const float phi_range = static_cast<float>(2.0 * M_PI),
                                     const Eigen::Isometry3d& origin = Eigen::Isometry3d::Identity());

/**
 * @details Creates a mesh of a cylinder with optional end caps
 * @param radius Radius (m) of the cylinder
 * @param length Length (m) of the cylinder
 * @param resolution Number of vertices around the perimeter of the cylinder
 * @param vertical_resolution Number of "rings" of vertices down the length of the cylinder (minimum 2)
 * @param theta_range Angle range (radians) of the cylinder, on (0, 2 * pi]. If the value is less than 2 * pi, a
 * cylinder shell is created
 * @param include_caps Flag to indicate whether the caps of the cylinder should be included
 * @param origin Transform to the desired origin of the primitive
 * @return
 */
pcl::PolygonMesh createCylinderMesh(const float radius,
                                    const float length,
                                    const std::size_t resolution = 20lu,
                                    std::size_t vertical_resolution = 2u,
                                    const float theta_range = static_cast<float>(2.0 * M_PI),
                                    const bool include_caps = true,
                                    const Eigen::Isometry3d& origin = Eigen::Isometry3d::Identity());

/**
 * @details Overload of @ref createCylinderMesh that creates a mesh of a cylinder with optional end caps and a minimal
 * number of triangles
 * @sa createCylinderMesh
 */
pcl::PolygonMesh createCylinderMesh(const float radius,
                                    const float length,
                                    const std::size_t resolution = 20lu,
                                    const float theta_range = static_cast<float>(2.0 * M_PI),
                                    const bool include_caps = true,
                                    const Eigen::Isometry3d& origin = Eigen::Isometry3d::Identity());

/**
 * @details Overload of @ref createCylinderMesh that creates a mesh of a cylinder with optional end caps and uniform
 * isoceles triangles
 * @sa createCylinderMesh
 */
pcl::PolygonMesh
createCylinderMeshWithUniformTriangles(const float radius,
                                       const float length,
                                       const std::size_t resolution = 20lu,
                                       const float theta_range = static_cast<float>(2.0 * M_PI),
                                       const bool include_caps = true,
                                       const Eigen::Isometry3d& origin = Eigen::Isometry3d::Identity());

}  // namespace noether
