#pragma once

#include <noether_tpp/core/mesh_modifier.h>
#include <noether_tpp/macros.h>

FWD_DECLARE_YAML_STRUCTS()

namespace noether
{
/**
 * @brief MeshModifier that applies iterative midpoint subdivision to create smaller mesh faces
 * @details This class subdivides faces by adding new vertices at the midpoints of edges of a face and creating smaller
 * new faces by connecting the midpoints and existing vertices. All new vertices added to the mesh retain the ancillary
 * data fields associated with the original vertices (e.g., color, normals, curvature, etc.). If normal and color
 * (`rgba`) data fields exists, added midpoints average the normal and color fields of the parent vertices.
 *
 * Note: midpoint subdivision increases the number of vertices and triangles by a factor of 4^n, so only a small number
 * of iterations is recommended.
 *
 * @ingroup mesh_modifiers
 */
class FaceMidpointSubdivisionMeshModifier : public MeshModifier
{
public:
  /**
   * @brief Constructor
   * @param n_iterations Number of iterations to perform midpoint subdivision
   */
  FaceMidpointSubdivisionMeshModifier(unsigned n_iterations);
  virtual ~FaceMidpointSubdivisionMeshModifier() = default;

  std::vector<pcl::PolygonMesh> modify(const pcl::PolygonMesh& mesh) const override;

protected:
  FaceMidpointSubdivisionMeshModifier() = default;
  DECLARE_YAML_FRIEND_CLASSES(FaceMidpointSubdivisionMeshModifier)

  /**
   * @brief Number of iterations of subdivision to run
   */
  unsigned n_iterations_;
};

/**
 * @brief MeshModifier that subdivides mesh faces until a user-defined criteria is met.
 * @details This class subdivides mesh faces by introducing a single new vertex in the middle of a triangle to create 3
 * new triangles per face. All new vertices added to the mesh retain the ancillary data fields associated with the
 * original vertices (e.g., color, normals, curvature, etc.). If normal and color (`rgba`) data fields exists, added
 * midpoints average the normal and color fields of the parent vertices.
 *
 * @note This class subdivides faces by inserting a vertex in the middle of the face, which means that all original
 * triangle edges are retained. Therefore, the criteria for face subdivision should **not** be based on a property of a
 * triangle edge (e.g., maximum allowable length).
 *
 * @ingroup mesh_modifiers
 */
class FaceSubdivisionMeshModifier : public MeshModifier
{
public:
  using MeshModifier::MeshModifier;
  virtual ~FaceSubdivisionMeshModifier() = default;

  std::vector<pcl::PolygonMesh> modify(const pcl::PolygonMesh& mesh) const override;

protected:
  /**
   * @details Checks if the provided mesh face requires subdivision.
   * @param mesh Input mesh (for access to vertices, etc.)
   * @param face Polygonal mesh face to check
   * @return True if the face should be subdivided, false otherwise
   */
  virtual bool requiresSubdivision(const pcl::PolygonMesh& mesh, const std::vector<pcl::index_t>& face) const = 0;
};

/**
 * @brief MeshModifier that subdivides mesh faces into smaller faces until all faces have less than the specified
 * maximum area
 * @ingroup mesh_modifiers
 */
class FaceSubdivisionByAreaMeshModifier : public FaceSubdivisionMeshModifier
{
public:
  FaceSubdivisionByAreaMeshModifier(float max_area);
  virtual ~FaceSubdivisionByAreaMeshModifier() = default;

protected:
  FaceSubdivisionByAreaMeshModifier() = default;
  DECLARE_YAML_FRIEND_CLASSES(FaceSubdivisionByAreaMeshModifier)

  /**
   * @brief Returns true if the area of the face has a greater area than the specified maximum face area
   */
  bool requiresSubdivision(const pcl::PolygonMesh& mesh, const std::vector<pcl::index_t>& face) const override;

  /**
   * @brief Maximum allowable face area (m^2)
   */
  float max_area_;
};

}  // namespace noether

FWD_DECLARE_YAML_CONVERT(noether::FaceMidpointSubdivisionMeshModifier)
FWD_DECLARE_YAML_CONVERT(noether::FaceSubdivisionByAreaMeshModifier)
