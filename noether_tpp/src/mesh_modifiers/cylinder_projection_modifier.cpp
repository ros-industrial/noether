#include <noether_tpp/mesh_modifiers/cylinder_projection_modifier.h>
#include <noether_tpp/mesh_modifiers/subset_extraction/subset_extractor.h>
#include <noether_tpp/serialization.h>
#include <noether_tpp/utils.h>

#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_cylinder.h>
#include <pcl/conversions.h>

namespace noether
{
/**
 * @brief Projects points onto the cylinder model in place
 * @details pcl::SampleConsensusModelCylinder has a bug in `projectPoints` that was fixed in 1.12.1.
 * To support back to PCL 1.10 (Ubuntu 20.04), we have implemented the projection function ourselves to include the bug
 * fix
 */
void projectInPlace(pcl::PCLPointCloud2& cloud, const Eigen::VectorXf& model_coefficients)
{
  Eigen::Vector4f line_pt(model_coefficients[0], model_coefficients[1], model_coefficients[2], 0.0f);
  Eigen::Vector4f line_dir(model_coefficients[3], model_coefficients[4], model_coefficients[5], 0.0f);
  float ptdotdir = line_pt.dot(line_dir);
  float dirdotdir = 1.0f / line_dir.dot(line_dir);

  // Create a helper function for projecting each point
  // https://github.com/PointCloudLibrary/pcl/blob/5f608cfb5397fe848c7d61c4ae5f5b1ab760ba80/sample_consensus/include/pcl/sample_consensus/impl/sac_model_cylinder.hpp#L398-L407
  auto project_point = [&](const Eigen::Vector4f& p) -> Eigen::Vector4f {
    Eigen::VectorXf pp;
    float k = (p.dot(line_dir) - ptdotdir) * dirdotdir;
    // Calculate the projection of the point on the line
    pp.matrix() = line_pt + k * line_dir;

    Eigen::Vector4f dir = p - pp;
    dir[3] = 0.0f;
    dir.normalize();

    // Calculate the projection of the point onto the cylinder
    pp += dir * model_coefficients[6];

    return pp;
  };

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
      p = project_point(p);

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

CylinderProjectionModifier::CylinderProjectionModifier(float min_radius,
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
CylinderProjectionModifier::createModel(const pcl::PolygonMesh& mesh) const
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
CylinderProjectionModifier::createSubMesh(const pcl::PolygonMesh& mesh,
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

}  // namespace noether

namespace YAML
{
/** @cond */
Node convert<noether::CylinderProjectionModifier>::encode(const noether::CylinderProjectionModifier& val)
{
  Node node = convert<noether::RansacPrimitiveFitMeshModifier>::encode(val);
  node["min_radius"] = val.min_radius_;
  node["max_radius"] = val.max_radius_;
  node["axis"] = val.axis_;
  node["axis_threshold"] = val.axis_threshold_;
  node["normal_distance_weight"] = val.normal_distance_weight_;

  return node;
}

bool convert<noether::CylinderProjectionModifier>::decode(const Node& node, noether::CylinderProjectionModifier& val)
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
