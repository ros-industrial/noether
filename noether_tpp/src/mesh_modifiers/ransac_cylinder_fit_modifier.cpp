#include <noether_tpp/mesh_modifiers/ransac_cylinder_fit_modifier.h>
#include <noether_tpp/mesh_modifiers/subset_extraction/subset_extractor.h>
#include <noether_tpp/serialization.h>
#include <noether_tpp/utils.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/conversions.h>

namespace noether
{
/**
 * @brief Helper class for performing projection of points onto a cylinder model
 * @details Implementation adapted from
 * https://github.com/PointCloudLibrary/pcl/blob/5f608cfb5397fe848c7d61c4ae5f5b1ab760ba80/sample_consensus/include/pcl/sample_consensus/impl/sac_model_cylinder.hpp#L398-L407
 */
class CylinderProjector
{
public:
  CylinderProjector(const Eigen::Ref<const Eigen::Vector3f>& line_pt,
                    const Eigen::Ref<const Eigen::Vector3f>& line_dir,
                    const float radius_)
    : line_pt_(line_pt)
    , line_dir_(line_dir)
    , radius_(radius_)
    , ptdotdir_(line_pt.dot(line_dir))
    , dirdotdir_(1.0f / line_dir.dot(line_dir))
  {
  }

  Eigen::Vector3f projectOntoAxis(const Eigen::Ref<const Eigen::Vector3f>& p)
  {
    const float k = (p.dot(line_dir_) - ptdotdir_) * dirdotdir_;
    return line_pt_ + k * line_dir_;
  }

  Eigen::Vector3f project(const Eigen::Ref<const Eigen::Vector3f>& p)
  {
    Eigen::Vector3f pp = projectOntoAxis(p);

    // Calculate the projection of the point onto the cylinder
    const Eigen::Vector3f dir = (p - pp).normalized();
    pp += dir * radius_;
    return pp;
  }

protected:
  const Eigen::Vector3f line_pt_;
  const Eigen::Vector3f line_dir_;
  const float radius_;
  const float ptdotdir_;
  const float dirdotdir_;
};

/**
 * @brief Projects points onto the cylinder model in-place
 */
void projectInPlace(pcl::PCLPointCloud2& cloud, const Eigen::VectorXf& model_coefficients)
{
  // Extract the model components
  Eigen::Vector3f line_pt(model_coefficients[0], model_coefficients[1], model_coefficients[2]);
  Eigen::Vector3f line_dir(model_coefficients[3], model_coefficients[4], model_coefficients[5]);
  float radius = model_coefficients(6);

  // Create the projection helper
  CylinderProjector projector(line_pt, line_dir, radius);

  // Find the x, y, and z fields
  auto x_it = findFieldOrThrow(cloud.fields, "x");
  auto y_it = findFieldOrThrow(cloud.fields, "y");
  auto z_it = findFieldOrThrow(cloud.fields, "z");

  // Check that the xyz fields are floats and contiguous
  if ((y_it->offset - x_it->offset != 4) || (z_it->offset - y_it->offset != 4))
    throw std::runtime_error("XYZ fields are not contiguous floats");

  // auto nx_it = findField(cloud.fields, "normal_x");
  // auto ny_it = findField(cloud.fields, "normal_y");
  // auto nz_it = findField(cloud.fields, "normal_z");

  // bool update_normals = nx_it != cloud.fields.end() && ny_it != cloud.fields.end() && nz_it != cloud.fields.end();

  for (std::size_t r = 0; r < cloud.height; ++r)
  {
    for (std::size_t c = 0; c < cloud.width; ++c)
    {
      auto offset = r * cloud.row_step + c * cloud.point_step;
      float* xyz = reinterpret_cast<float*>(cloud.data.data() + offset + x_it->offset);
      Eigen::Map<Eigen::Vector4f> p(xyz);

      // Project the point in place
      p.head<3>() = projector.project(p.head<3>());

      // TODO: update calculation of normals
      // if (update_normals)
      // {
      //   float* nx = reinterpret_cast<float*>(cloud.data.data() + offset + nx_it->offset);
      //   float* ny = reinterpret_cast<float*>(cloud.data.data() + offset + ny_it->offset);
      //   float* nz = reinterpret_cast<float*>(cloud.data.data() + offset + nz_it->offset);
      // }
    }
  }
}

RansacCylinderProjectionMeshModifier::RansacCylinderProjectionMeshModifier(float min_radius,
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
RansacCylinderProjectionMeshModifier::createModel(const pcl::PolygonMesh& mesh) const
{
  if (!hasNormals(mesh.cloud))
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

pcl::PolygonMesh RansacCylinderProjectionMeshModifier::createSubMesh(
    const pcl::PolygonMesh& mesh,
    std::shared_ptr<const pcl::RandomSampleConsensus<pcl::PointXYZ>> ransac) const
{
  Eigen::VectorXf coefficients;
  ransac->getModelCoefficients(coefficients);

  std::vector<int> inliers;
  ransac->getInliers(inliers);

  // Extract the inlier submesh
  pcl::PolygonMesh output_mesh = extractSubMeshFromInlierVertices(mesh, inliers);

  // Project the mesh
  projectInPlace(output_mesh.cloud, coefficients);

  return output_mesh;
}

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

RansacCylinderFitMeshModifier::RansacCylinderFitMeshModifier(float min_radius,
                                                             float max_radius,
                                                             float distance_threshold,
                                                             const Eigen::Vector3f& axis,
                                                             float axis_threshold,
                                                             float normal_distance_weight,
                                                             unsigned min_vertices,
                                                             int max_cylinders,
                                                             unsigned max_iterations,
                                                             unsigned primitive_resolution,
                                                             bool include_primitive_caps)
  : RansacCylinderProjectionMeshModifier(min_radius,
                                         max_radius,
                                         distance_threshold,
                                         axis,
                                         axis_threshold,
                                         normal_distance_weight,
                                         min_vertices,
                                         max_cylinders,
                                         max_iterations)
  , resolution_(std::max(6u, primitive_resolution))
  , include_caps_(include_primitive_caps)
{
}

pcl::PolygonMesh RansacCylinderFitMeshModifier::createSubMesh(
    const pcl::PolygonMesh& mesh,
    std::shared_ptr<const pcl::RandomSampleConsensus<pcl::PointXYZ>> ransac) const
{
  Eigen::VectorXf coefficients;
  ransac->getModelCoefficients(coefficients);

  std::vector<int> inliers;
  ransac->getInliers(inliers);

  // Extract the cylinder axis and radius
  const Eigen::Vector3f pt_on_axis = coefficients.head<3>();
  const Eigen::Vector3f axis = coefficients.block(3, 0, 3, 1);
  const float radius = coefficients(6);

  // Create the projection helper
  CylinderProjector projector(pt_on_axis, axis, radius);

  // Extract the inlier vertices
  const auto vertices = ransac->getSampleConsensusModel()->getInputCloud()->getMatrixXfMap()({ 0, 1, 2 }, inliers);

  // Compute the length of the cylinder
  // Project the vertices onto the cylinder axis; subtract the max projection from the min projection
  const Eigen::VectorXf vertex_projections = (vertices.transpose() * axis) / axis.norm();
  const float length = vertex_projections.maxCoeff() - vertex_projections.minCoeff();

  // Compute the origin of the cylinder
  const Eigen::Vector3f centroid = vertices.rowwise().mean();
  const Eigen::Vector3f origin = projector.projectOntoAxis(centroid);

  // Use the centroid and the cylinder axis to create the origin offset transform
  const Eigen::Isometry3d transform = createTransform(origin.cast<double>(), axis.cast<double>());

  // Create the cylinder primitive
  return createCylinderMesh(radius, length, resolution_, 2.0 * M_PI, include_caps_, transform);
}

}  // namespace noether

namespace YAML
{
/** @cond */
Node convert<noether::RansacCylinderProjectionMeshModifier>::encode(
    const noether::RansacCylinderProjectionMeshModifier& val)
{
  Node node = convert<noether::RansacPrimitiveFitMeshModifier>::encode(val);
  node["min_radius"] = val.min_radius_;
  node["max_radius"] = val.max_radius_;
  node["axis"] = val.axis_;
  node["axis_threshold"] = val.axis_threshold_;
  node["normal_distance_weight"] = val.normal_distance_weight_;

  return node;
}

bool convert<noether::RansacCylinderProjectionMeshModifier>::decode(const Node& node,
                                                                    noether::RansacCylinderProjectionMeshModifier& val)
{
  bool ret = convert<noether::RansacPrimitiveFitMeshModifier>::decode(node, val);

  val.min_radius_ = YAML::getMember<double>(node, "min_radius");
  val.max_radius_ = YAML::getMember<double>(node, "max_radius");
  val.axis_ = YAML::getMember<Eigen::Vector3f>(node, "axis");
  val.axis_threshold_ = YAML::getMember<double>(node, "axis_threshold");
  val.normal_distance_weight_ = YAML::getMember<double>(node, "normal_distance_weight");

  return ret;
}

Node convert<noether::RansacCylinderFitMeshModifier>::encode(const noether::RansacCylinderFitMeshModifier& val)
{
  Node node = convert<noether::RansacCylinderProjectionMeshModifier>::encode(val);
  node["resolution"] = val.resolution_;
  node["include_caps"] = val.include_caps_;
  return node;
}

bool convert<noether::RansacCylinderFitMeshModifier>::decode(const Node& node,
                                                             noether::RansacCylinderFitMeshModifier& val)
{
  bool ret = convert<noether::RansacCylinderProjectionMeshModifier>::decode(node, val);
  val.resolution_ = YAML::getMember<unsigned>(node, "resolution");
  val.include_caps_ = YAML::getMember<bool>(node, "include_caps");
  return ret;
}
/** @endcond */

}  // namespace YAML
