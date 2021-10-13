#pragma once

#include <memory>

#include <noether_tpp/tool_path_planners/raster/raster_planner.h>

namespace noether
{
/**
 * @brief Returns a fixed point as the origin
 * @details The default fixed point is a zero vector corresponding to the mesh coordinate system origin
 */
class FixedOriginGenerator : public OriginGenerator
{
public:
  FixedOriginGenerator(const Eigen::Vector3d& origin = Eigen::Vector3d::Zero());
  Eigen::Vector3d generate(const pcl::PolygonMesh& mesh) const override final;
  std::unique_ptr<OriginGenerator> clone() const override final;

private:
  const Eigen::Vector3d origin_;
};

/**
 * @brief Returns the centroid of the mesh vertices as the origin
 */
struct CentroidOriginGenerator : public OriginGenerator
{
  Eigen::Vector3d generate(const pcl::PolygonMesh& mesh) const override final;
  std::unique_ptr<OriginGenerator> clone() const override final;
};

/**
 * @brief Returns the center of the axis-aligned bounding box (AABB) as the origin
 */
struct AABBCenterOriginGenerator : public OriginGenerator
{
  Eigen::Vector3d generate(const pcl::PolygonMesh& mesh) const override final;
  std::unique_ptr<OriginGenerator> clone() const override final;
};

/**
 * @brief Returns the origin as the origin of another specified origin generator plus a fixed offset
 */
class OffsetOriginGenerator : public OriginGenerator
{
public:
  OffsetOriginGenerator(OriginGenerator::ConstPtr&& generator, const Eigen::Vector3d& offset);
  Eigen::Vector3d generate(const pcl::PolygonMesh& mesh) const override final;
  std::unique_ptr<OriginGenerator> clone() const override final;

private:
  const OriginGenerator::ConstPtr generator_;
  const Eigen::Vector3d offset_;
};

}  // namespace noether
