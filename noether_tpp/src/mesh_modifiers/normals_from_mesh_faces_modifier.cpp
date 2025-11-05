#include <noether_tpp/mesh_modifiers/normals_from_mesh_faces_modifier.h>
#include <noether_tpp/utils.h>

#include <vector>

#include <Eigen/Dense>
#include <pcl/common/io.h>
#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/PolygonMesh.h>
#include <yaml-cpp/yaml.h>

namespace noether
{

/**
 * @brief calculateNormalFromThreePoints - Calculate the normal of a face based on three vertices,
 * traversing in counter-clockwise order
 * @param a - input - first vertex
 * @param b - input - second vertex
 * @param c - input - third vertex
 * @return Eigen::Vector3d representing the face's normal
 */
Eigen::Vector3d calculateNormalFromThreePoints(
    const pcl::PointXYZ& a,
    const pcl::PointXYZ& b,
    const pcl::PointXYZ& c)
{
  // Convert the three points to Eigen::Vector3d
  Eigen::Vector3d ae (a.x, a.y, a.z);
  Eigen::Vector3d be (b.x, b.y, b.z);
  Eigen::Vector3d ce (c.x, c.y, c.z);

  // Calculate the vectors that represent two sides of a triangle
  Eigen::Vector3d ab = be - ae;
  Eigen::Vector3d ac = ce - ae;

  // Calculate the face normal as the cross product of two sides
  Eigen::Vector3d normal = ab.cross(ac).normalized();
  return normal;
}

/**
 * @brief getNormalsFromMeshWithoutDuplicatePoints - Calculate the normals of all vertices in a
 * polygon mesh. The normal of each vertex is calculated as the average of the normals of all faces
 * which include that vertex. This function handles supra-triangular polygonal faces, but ignores
 * 'faces' that include less than three vertices. Well-behaved on meshes with holes. Note that a
 * mesh with inconsistent face normals will generate inconsistent vertex normals. Note that a
 * non-manifold mesh may produce numerically unstable results.
 * @param mesh - input - a polygonal mesh.
 * @return std::vector<Eigen::Vector3d> containing one normal for each vertex in the input mesh, in
 * the same order as the input mesh's vertex list
 */
std::vector<Eigen::Vector3d> getNormalsFromMeshWithoutDuplicatePoints(const pcl::PolygonMesh& mesh)
{
  // Pull the point cloud out of the mesh
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromPCLPointCloud2(mesh.cloud, cloud);

  // Make acculumulators for normal calculations
  std::vector<Eigen::Vector3d> normals (cloud.size(), Eigen::Vector3d::Zero());

  // Iterate across the polygons, calculating their normals and accumulating the normals onto all
  // adjacent vertices
  for (const auto& p : mesh.polygons)
  {
    // Ensure this polygon has at least three vertices, as
    // we will use the first three to calculate the normal
    if (p.vertices.size() < 3)
    {
      // Throw an exception here if desired, otherwise skip this face
      continue;
    }

    // Ensure the first three vertex references are valid,
    // since we will use these when calculating the normal
    if (p.vertices[0] >= cloud.size() ||
        p.vertices[1] >= cloud.size() ||
        p.vertices[2] >= cloud.size())
    {
      // Throw an exception here if you want, otherwise skip this face
      continue;
    }

    // Calculate the normal for this face with helper function
    Eigen::Vector3d normal = calculateNormalFromThreePoints(
        cloud[p.vertices[0]],
        cloud[p.vertices[1]],
        cloud[p.vertices[2]]);

    // Add this normal to the accumulators for this face's valid vertices
    for (std::size_t i = 0; i < p.vertices.size(); ++i)
    {
      if (p.vertices[i] < cloud.size())
      {
        normals[p.vertices[i]] += normal;
      }
    }
  }

  // No need to divide by the number of contributing faces. Since we still need to normalize() to
  // unit length afterward, dividing by a non-zero finite scalar first has no effect.
  for (std::size_t i = 0; i < normals.size(); ++i)
  {
    if (normals[i].norm() >= 0.0001) // make sure that a normal was ever added here
    {
      normals[i] = normals[i].normalized();
    }
  }

  return normals;
} // function getNormalsFromMeshWithoutDuplicatePoints()

std::vector<pcl::PolygonMesh> NormalsFromMeshFacesMeshModifier::modify(const pcl::PolygonMesh& mesh) const
{
  // Calculate the normals
  std::vector<Eigen::Vector3d> normals = getNormalsFromMeshWithoutDuplicatePoints(mesh);

  // Reserve space for PCL representation of normals
  pcl::PointCloud<pcl::Normal> normals_cloud;
  normals_cloud.reserve(normals.size());

  // Convert normals to PCL representation
  for (const auto& normal_pt : normals)
  {
    pcl::Normal normal;
    normal.getNormalVector3fMap() = normal_pt.cast<float>().normalized();
    normals_cloud.push_back(normal);
  }

  // Convert to message type
  pcl::PCLPointCloud2 normals_pcl2;
  pcl::toPCLPointCloud2(normals_cloud, normals_pcl2);

  // Create output mesh
  pcl::PolygonMesh output = mesh;

  // Concatenate the normals with the nominal mesh information
  if (!pcl::concatenateFields(mesh.cloud, normals_pcl2, output.cloud))
    throw std::runtime_error("Failed to concatenate normals into mesh vertex cloud");

  return { output };
} // function modify()

}  // namespace noether

namespace YAML
{
/** @cond */
Node convert<noether::NormalsFromMeshFacesMeshModifier>::encode(const noether::NormalsFromMeshFacesMeshModifier& val)
{
  return {};
}

bool convert<noether::NormalsFromMeshFacesMeshModifier>::decode(const Node& node,
                                                                noether::NormalsFromMeshFacesMeshModifier& val)
{
  return true;
}
/** @endcond */

}  // namespace YAML
