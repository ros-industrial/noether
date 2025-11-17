#pragma once

#include <noether_tpp/mesh_modifiers/ransac_primitive_fit_modifier.h>

FWD_DECLARE_YAML_STRUCTS()

namespace noether
{
/**
 * @ingroup mesh_modifiers
 * @brief MeshModifier that fits a specifiable number of cylinders to the input mesh and projects the inlier vertices
 * onto the fitted cylinder models
 * @details This modifier uses RANSAC to sequentially fit cylinder models to the input mesh.
 * When a valid cylinder model is identified, the inlier vertices are removed from the input mesh into a new submesh and
 * then projected onto the fitted cylinder model. New cylinder models are then fitted to the remainder of vertices in
 * the input mesh until either 1) the maximum number of cylinder models have been fitted 2) too few inliers remain in
 * the input mesh, or 3) no more valid cylinders can be fit to the input mesh.
 */
class RansacCylinderProjectionMeshModifier : public RansacPrimitiveFitMeshModifier
{
public:
  /**
   * @brief Constructor
   * @param min_radius Minimum required cylinder radius (m)
   * @param max_radius Maximum required cylinder radius (m)
   * @param distance_threshold Maximum distance (m) a point can be from a model of a cylinder to be considered an inlier
   * @param axis Predicted cylinder axis
   * @param axis_threshold Maximum angle (radians) by which the axis of a detected cylinder can differ from the axis of
   * the defined model cylinder
   * @param normal_distance_weight Distance weighting amount given to normals (vs point positions) when comparing to the
   * cylinder model (value from [0, 1])
   * @param min_vertices Minimum number of vertices that a cluster (identfied as a cylinder) must have
   * @param max_cylinders Maximum number of cylinders to detect
   * @param max_iterations Maximum number of RANSAC iterations to perform
   */
  RansacCylinderProjectionMeshModifier(float min_radius,
                                       float max_radius,
                                       float distance_threshold,
                                       const Eigen::Vector3f& axis = Eigen::Vector3f::UnitZ(),
                                       float axis_threshold = 10.0 * M_PI / 180.0,
                                       float normal_distance_weight = 0.1,
                                       unsigned min_vertices = 1,
                                       int max_cylinders = -1,
                                       unsigned max_iterations = 100);

protected:
  RansacCylinderProjectionMeshModifier() = default;
  DECLARE_YAML_FRIEND_CLASSES(RansacCylinderProjectionMeshModifier)

  std::shared_ptr<pcl::SampleConsensusModel<pcl::PointXYZ>> createModel(const pcl::PolygonMesh& mesh) const override;

  pcl::PolygonMesh
  createSubMesh(const pcl::PolygonMesh& mesh,
                std::shared_ptr<const pcl::RandomSampleConsensus<pcl::PointXYZ>> ransac) const override;

  float min_radius_;
  float max_radius_;
  Eigen::Vector3f axis_;
  float axis_threshold_;
  float normal_distance_weight_;
};

/**
 * @ingroup mesh_modifiers
 * @brief MeshModifier that fits a specifiable number of cylinders to the input mesh and returns the fitted primitive
 * models
 * @details This modifier uses RANSAC to sequentially fit cylinder models to the input mesh.
 * When a valid cylinder model is identified, the inlier vertices are removed from the input mesh and the fitted
 * primitive model is returned. New cylinder models are then fitted to the remainder of vertices in the input mesh until
 * either 1) the maximum number of cylinder models have been fitted 2) too few inliers remain in the input mesh, or 3)
 * no more valid cylinders can be fit to the input mesh.
 */
class RansacCylinderFitMeshModifier : public RansacCylinderProjectionMeshModifier
{
public:
  /**
   * @brief Constructor
   * @param min_radius Minimum required cylinder radius (m)
   * @param max_radius Maximum required cylinder radius (m)
   * @param distance_threshold Maximum distance (m) a point can be from a model of a cylinder to be considered an inlier
   * @param axis Predicted cylinder axis
   * @param axis_threshold Maximum angle (radians) by which the axis of a detected cylinder can differ from the axis of
   * the defined model cylinder
   * @param normal_distance_weight Distance weighting amount given to normals (vs point positions) when comparing to the
   * cylinder model (value from [0, 1])
   * @param min_vertices Minimum number of vertices that a cluster (identfied as a cylinder) must have
   * @param max_cylinders Maximum number of cylinders to detect
   * @param max_iterations Maximum number of RANSAC iterations to perform
   */
  RansacCylinderFitMeshModifier(float min_radius,
                                float max_radius,
                                float distance_threshold,
                                const Eigen::Vector3f& axis = Eigen::Vector3f::UnitZ(),
                                float axis_threshold = 10.0 * M_PI / 180.0,
                                float normal_distance_weight = 0.1,
                                unsigned min_vertices = 1,
                                int max_cylinders = -1,
                                unsigned max_iterations = 100);

protected:
  RansacCylinderFitMeshModifier() = default;
  DECLARE_YAML_FRIEND_CLASSES(RansacCylinderFitMeshModifier)

  pcl::PolygonMesh
  createSubMesh(const pcl::PolygonMesh& mesh,
                std::shared_ptr<const pcl::RandomSampleConsensus<pcl::PointXYZ>> ransac) const override;
};

}  // namespace noether

FWD_DECLARE_YAML_CONVERT(noether::RansacCylinderProjectionMeshModifier)
FWD_DECLARE_YAML_CONVERT(noether::RansacCylinderFitMeshModifier)
