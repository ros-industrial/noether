/**
 * @author Jorge Nicho <jrgnichodevel@gmail.com>
 * @file bspline_reconstruction_modifier.cpp
 * @date Oct 10, 2019
 * @copyright Copyright (c) 2019, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <noether_tpp/mesh_modifiers/bspline_reconstruction_modifier.h>
#include <pcl/conversions.h>
#include <pcl/surface/on_nurbs/triangulation.h>

static pcl::on_nurbs::vector_vec3d createNurbData(pcl::PointCloud<pcl::PointXYZ>::ConstPtr cloud)
{
  pcl::on_nurbs::vector_vec3d data;
  for (unsigned i = 0; i < cloud->size(); i++)
  {
    const pcl::PointXYZ& p = cloud->at(i);
    if (!std::isnan(p.x) && !std::isnan(p.y) && !std::isnan(p.z))
      data.push_back(Eigen::Vector3d(p.x, p.y, p.z));
  }
  return std::move(data);
}

namespace noether
{
std::vector<pcl::PolygonMesh> BSplineReconstruction::modify(const pcl::PolygonMesh& mesh_in) const
{
  using Cloud = pcl::PointCloud<pcl::PointXYZ>;

  // converting to point cloud
  Cloud::Ptr cloud_in = boost::make_shared<Cloud>();
  pcl::fromPCLPointCloud2(mesh_in.cloud, *cloud_in);

  // initializing nurbs_surface surface
  pcl::on_nurbs::NurbsDataSurface nurbs_data = pcl::on_nurbs::NurbsDataSurface();
  nurbs_data.interior = createNurbData(cloud_in);
  ON_NurbsSurface nurbs_surface;
  switch (parameters_.surf_init_method)
  {
    case SurfInitMethod::PCA:
      nurbs_surface =
          pcl::on_nurbs::FittingSurface::initNurbsPCA(parameters_.order, &nurbs_data, Eigen::Vector3d::UnitZ());
      break;
    case SurfInitMethod::PCA_BB:
      nurbs_surface = pcl::on_nurbs::FittingSurface::initNurbsPCABoundingBox(
          parameters_.order, &nurbs_data, Eigen::Vector3d::UnitZ());
      break;
    case SurfInitMethod::CUSTOM_PLANE:
      throw std::runtime_error("The surface initialization method 'CUSTOM_PLANE' is not yet implemented");
    default:
      throw std::runtime_error("The surface initialization method was not recognized");
  }

  // fitting surface
  pcl::on_nurbs::FittingSurface fit(&nurbs_data, nurbs_surface);
  fit.setQuiet(!parameters_.verbosity_on);  // enable/disable debug output
  pcl::on_nurbs::FittingSurface::Parameter surf_params = parameters_.surface_params;
  for (int i = 0; i < parameters_.refinement; i++)
  {
    fit.refine(0);
    fit.refine(1);
    fit.assemble(surf_params);
    fit.solve();
  }

  // improving fit
  for (unsigned i = 0; i < parameters_.iterations; i++)
  {
    fit.assemble(surf_params);
    fit.solve();
  }

  if (!fit.m_nurbs.IsValid())
    throw std::runtime_error("Surface fitting failed");

  // fit B-spline boundary curve
  std::shared_ptr<pcl::on_nurbs::FittingCurve2dASDM> curve_fit = nullptr;
  if (parameters_.clip_boundary_curve)
  {
    // initialisation (circular)
    pcl::on_nurbs::NurbsDataCurve2d curve_data;
    curve_data.interior = nurbs_data.interior_param;
    curve_data.interior_weight_function.push_back(true);
    ON_NurbsCurve curve_nurbs = pcl::on_nurbs::FittingCurve2dAPDM::initNurbsCurve2D(
        parameters_.boundary_fit_order, curve_data.interior, parameters_.boundary_startCPs);

    // curve fitting
    curve_fit = std::make_shared<pcl::on_nurbs::FittingCurve2dASDM>(&curve_data, curve_nurbs);
    curve_fit->setQuiet(!parameters_.verbosity_on);  // enable/disable debug output
    curve_fit->fitting(parameters_.boundary_curve_params);

    if (!curve_fit->m_nurbs.IsValid() && parameters_.boundary_clipping_required)
      throw std::runtime_error("Failed to fit boundary curve");
  }

  pcl::PolygonMesh mesh_out;
  if (curve_fit && curve_fit->m_nurbs.IsValid())
  {
    pcl::on_nurbs::Triangulation::convertTrimmedSurface2PolygonMesh(
        fit.m_nurbs, curve_fit->m_nurbs, mesh_out, parameters_.mesh_resolution);
  }
  else
  {
    pcl::on_nurbs::Triangulation::convertSurface2PolygonMesh(fit.m_nurbs, mesh_out, parameters_.mesh_resolution);
  }

  return { mesh_out };
}

}  // namespace noether
