#pragma once

#include <noether_tpp/core/mesh_modifier.h>
#include <noether_tpp/macros.h>

FWD_DECLARE_YAML_STRUCTS()

namespace noether
{
/**
 * @brief Mesh modifiers that subdivides mesh faces into smaller faces until a user-specified criteria is met
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
   * @brief Checks if the provided mesh face requires subdivision
   * @param mesh Input mesh (for access to vertices, etc.)
   * @param face Polygonal mesh face to check
   * @return True if the face should be subdivided, false otherwise
   */
  virtual bool requiresSubdivision(const pcl::PolygonMesh& mesh, const pcl::Vertices& face) const = 0;
};

/**
 * @brief Mesh modifier that subdivides mesh faces into smaller faces until all faces have less than the specified maximum area
 * @ingroup mesh_modifiers
 */
class FaceSubdivisionByAreaMeshModifier : public FaceSubdivisionMeshModifier
{
public:
  FaceSubdivisionByAreaMeshModifier(float max_area);

protected:
  FaceSubdivisionByAreaMeshModifier() = default;
  DECLARE_YAML_FRIEND_CLASSES(FaceSubdivisionByAreaMeshModifier)

  /**
   * @brief Returns true if the area of the face has a greater area than the specified maximum face area
   */
  bool requiresSubdivision(const pcl::PolygonMesh& mesh, const pcl::Vertices& face) const override;

  float max_area_;
};

/**
 * @brief Mesh modifier that subdivides mesh faces into smaller faces until all edges have less than the specified maximum length
 * @ingroup mesh_modifiers
 */
class FaceSubdivisionByEdgeLengthMeshModifier : public FaceSubdivisionMeshModifier
{
public:
  FaceSubdivisionByEdgeLengthMeshModifier(float max_edge_length,
                                          float min_edge_length = 1.0e-6);

protected:
  FaceSubdivisionByEdgeLengthMeshModifier() = default;
  DECLARE_YAML_FRIEND_CLASSES(FaceSubdivisionByEdgeLengthMeshModifier)

  /**
   * @brief Returns true if the area of the face has a greater area than the specified maximum face area
   */
  bool requiresSubdivision(const pcl::PolygonMesh& mesh, const pcl::Vertices& face) const override;

  float max_edge_length_;
  float min_edge_length_x2_;
};

}  // namespace noether

FWD_DECLARE_YAML_CONVERT(noether::FaceSubdivisionByAreaMeshModifier)
FWD_DECLARE_YAML_CONVERT(noether::FaceSubdivisionByEdgeLengthMeshModifier)
