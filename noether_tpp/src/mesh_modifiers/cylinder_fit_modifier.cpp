#include <noether_tpp/mesh_modifiers/cylinder_fit_modifier.h>
#include <noether_tpp/serialization.h>
#include <noether_tpp/utils.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/conversions.h>

namespace noether
{
/**
 * @brief Creates a transform from a position and normal vector (which represents the z-axis)
 */
static Eigen::Isometry3d createTransform(const Eigen::Vector3d& position, const Eigen::Vector3d& normal)
{
  Eigen::Vector3d x_axis;
  Eigen::Vector3d y_axis;
  if (std::abs(normal.dot(Eigen::Vector3d::UnitX())) < 1.0e-3)
  {
    y_axis = normal.cross(Eigen::Vector3d::UnitX());
    x_axis = y_axis.cross(normal);
  }
  else
  {
    x_axis = Eigen::Vector3d::UnitY().cross(normal);
    y_axis = normal.cross(x_axis);
  }

  Eigen::Isometry3d origin = Eigen::Isometry3d::Identity();
  origin.matrix().col(0).head<3>() = x_axis.normalized();
  origin.matrix().col(1).head<3>() = y_axis.normalized();
  origin.matrix().col(2).head<3>() = normal.normalized();
  origin.matrix().col(3).head<3>() = position;

  return origin;
}

CylinderFitModifier::CylinderFitModifier(float min_radius,
                                         float max_radius,
                                         float distance_threshold,
                                         const Eigen::Vector3f& axis,
                                         float axis_threshold,
                                         float normal_distance_weight,
                                         unsigned min_vertices,
                                         int max_cylinders,
                                         unsigned max_iterations)
  : RansacPrimitiveFitMeshModifier(distance_threshold, min_vertices, max_cylinders, max_iterations)
  , min_radius_(min_radius)
  , max_radius_(max_radius)
  , axis_(axis)
  , axis_threshold_(axis_threshold)
  , normal_distance_weight_(normal_distance_weight)
{
}

std::shared_ptr<pcl::SampleConsensusModel<pcl::PointXYZ>>
CylinderFitModifier::createModel(const pcl::PolygonMesh& mesh) const
{
  if (!hasNormals(mesh))
    throw std::runtime_error("Cylinder fit requires mesh to have vertex normals");

  // Convert the mesh vertices to a point cloud
  auto cloud_points = pcl::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
  pcl::fromPCLPointCloud2(mesh.cloud, *cloud_points);

  // Convert the mesh vertex normals to a point cloud of normals
  auto cloud_normals = pcl::make_shared<pcl::PointCloud<pcl::Normal>>();
  pcl::fromPCLPointCloud2(mesh.cloud, *cloud_normals);

  // Set up the RANSAC cylinder fit model
  auto model = pcl::make_shared<pcl::SampleConsensusModelCylinder<pcl::PointXYZ, pcl::Normal>>(cloud_points);
  model->setInputNormals(cloud_normals);
  model->setNormalDistanceWeight(normal_distance_weight_);
  model->setRadiusLimits(min_radius_, max_radius_);
  model->setAxis(axis_);
  model->setEpsAngle(axis_threshold_);

  return model;
}

pcl::PolygonMesh
CylinderFitModifier::createSubMesh(const pcl::PolygonMesh& mesh,
                                   std::shared_ptr<const pcl::RandomSampleConsensus<pcl::PointXYZ>> ransac) const
{
  Eigen::VectorXf coefficients;
  ransac->getModelCoefficients(coefficients);

  std::vector<int> inliers;
  ransac->getInliers(inliers);

  // Extract the cylinder axis and radius
  const Eigen::Vector3f pt_on_axis = coefficients.head<3>();
  const Eigen::Vector3f axis_dir = coefficients.block(3, 0, 3, 1).normalized();
  const float radius = coefficients(6);

  // Extract the inlier vertices
  auto vertices = ransac->getSampleConsensusModel()->getInputCloud()->getMatrixXfMap()({ 0, 1, 2 }, inliers);

  // Compute the centroid of the inliers
  Eigen::Vector3f centroid = vertices.rowwise().mean();
  Eigen::Vector3f axis_normal = pt_on_axis - (pt_on_axis.dot(axis_dir) * axis_dir);
  Eigen::Vector3f projected_centroid = centroid.dot(axis_dir) * axis_dir + axis_normal;

  // Compute the length of the cylinder
  // Project the vertices onto the cylinder axis; subtract the max projection from the min projection
  Eigen::VectorXf dists = vertices.transpose() * axis_dir;
  float length = dists.maxCoeff() - dists.minCoeff();

  // Use the centroid and the cylinder axis to create the origin offset transform
  Eigen::Isometry3d origin = createTransform(projected_centroid.cast<double>(), axis_dir.cast<double>());

  // Create the cylinder primitive
  return createCylinderMesh(radius, length, 50, 2.0 * M_PI, false, origin);
}

}  // namespace noether

namespace YAML
{
/** @cond */
Node convert<noether::CylinderFitModifier>::encode(const noether::CylinderFitModifier& val)
{
  Node node = convert<noether::RansacPrimitiveFitMeshModifier>::encode(val);
  node["min_radius"] = val.min_radius_;
  node["max_radius"] = val.max_radius_;
  node["axis"] = val.axis_;
  node["axis_threshold"] = val.axis_threshold_;
  node["normal_distance_weight"] = val.normal_distance_weight_;

  return node;
}

bool convert<noether::CylinderFitModifier>::decode(const Node& node, noether::CylinderFitModifier& val)
{
  bool ret = convert<noether::RansacPrimitiveFitMeshModifier>::decode(node, val);

  val.min_radius_ = YAML::getMember<double>(node, "min_radius");
  val.max_radius_ = YAML::getMember<double>(node, "max_radius");
  val.axis_ = YAML::getMember<Eigen::Vector3f>(node, "axis");
  val.axis_threshold_ = YAML::getMember<double>(node, "axis_threshold");
  val.normal_distance_weight_ = YAML::getMember<double>(node, "normal_distance_weight");

  return ret;
}
/** @endcond */

}  // namespace YAML
