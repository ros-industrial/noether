#include <noether_tpp/mesh_modifiers/normals_from_mesh_faces_modifier.h>
#include <noether_tpp/utils.h>

#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/conversions.h>

namespace noether
{
Eigen::Vector3f computeFaceNormal(const pcl::PolygonMesh& mesh,
                                  const TriangleMesh& tri_mesh,
                                  const TriangleMesh::FaceIndex& face_idx)
{
  using VAFC = TriangleMesh::VertexAroundFaceCirculator;

  // Get the vertices of this triangle
  VAFC v_circ = tri_mesh.getVertexAroundFaceCirculator(face_idx);

  const TriangleMesh::VertexIndex v1_idx = v_circ++.getTargetIndex();
  const TriangleMesh::VertexIndex v2_idx = v_circ++.getTargetIndex();
  const TriangleMesh::VertexIndex v3_idx = v_circ++.getTargetIndex();

  // Check the validity of the vertices
  if (!v1_idx.isValid() || !v2_idx.isValid() || !v3_idx.isValid())
    return Eigen::Vector3f::Constant(std::numeric_limits<float>::quiet_NaN());

  const Eigen::Vector3f v1 = getPoint(mesh.cloud, v1_idx.get());
  const Eigen::Vector3f v2 = getPoint(mesh.cloud, v2_idx.get());
  const Eigen::Vector3f v3 = getPoint(mesh.cloud, v3_idx.get());

  // Get the edges v1 -> v2 and v1 -> v3
  const Eigen::Vector3f edge_12 = v2 - v1;
  const Eigen::Vector3f edge_13 = v3 - v1;

  // Compute the face normal as the cross product between edge v1 -> v2 and edge v1 -> v3
  return edge_12.cross(edge_13).normalized();
}

std::vector<pcl::PolygonMesh> NormalsFromMeshFacesMeshModifier::modify(const pcl::PolygonMesh& mesh) const
{
  // Create a triangle mesh representation of the input mesh
  TriangleMesh tri_mesh = createTriangleMesh(mesh);

  pcl::PointCloud<pcl::Normal> normals_cloud;
  normals_cloud.reserve(mesh.cloud.height * mesh.cloud.width);

  // For each vertex, circulate the faces around and average the face normals
  for (std::size_t i = 0; i < tri_mesh.getVertexDataCloud().size(); ++i)
  {
    Eigen::Vector3f avg_face_normal = Eigen::Vector3f::Zero();
    int n_faces = 0;

    using FAVC = TriangleMesh::FaceAroundVertexCirculator;
    FAVC circ = tri_mesh.getFaceAroundVertexCirculator(TriangleMesh::VertexIndex(i));

    FAVC circ_end = circ;
    do
    {
      if (n_faces > mesh.polygons.size())
        throw std::runtime_error("Vertex " + std::to_string(i) +
                                 " appears to participate in more faces than exist in the mesh; this mesh likely "
                                 "contains degenerate half-edges.");

      if (circ.isValid())
      {
        TriangleMesh::FaceIndex face_idx = circ.getTargetIndex();
        if (face_idx.isValid())
        {
          avg_face_normal += computeFaceNormal(mesh, tri_mesh, face_idx);
          ++n_faces;
        }
      }
    } while (++circ != circ_end);

    pcl::Normal normal;
    normal.getNormalVector3fMap() = (avg_face_normal / n_faces).normalized();
    normals_cloud.push_back(normal);
  }

  // Convert to message type
  pcl::PCLPointCloud2 normals;
  pcl::toPCLPointCloud2(normals_cloud, normals);

  // Copy the mesh
  pcl::PolygonMesh output = mesh;

  // Concatenate the normals with the nominal mesh information
  if (!pcl::concatenateFields(mesh.cloud, normals, output.cloud))
    throw std::runtime_error("Failed to concatenate normals into mesh vertex cloud");

  return { output };
}

}  // namespace noether
