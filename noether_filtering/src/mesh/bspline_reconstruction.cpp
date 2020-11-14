/**
 * @author Jorge Nicho <jrgnichodevel@gmail.com>
 * @file bspline_reconstruction.cpp
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

#include "noether_filtering/mesh/bspline_reconstruction.h"
#include "noether_filtering/utils.h"
#include <boost/make_shared.hpp>
#include <console_bridge/console.h>
#include <pcl/conversions.h>
#include <pcl/surface/on_nurbs/triangulation.h>
#include <XmlRpcException.h>

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

namespace noether_filtering
{
namespace mesh
{
bool BSplineReconstruction::configure(XmlRpc::XmlRpcValue config)
{
  Parameters& p = parameters_;
  try
  {
    p.verbosity_on = static_cast<bool>(config["verbosity_on"]);
    p.order = static_cast<int>(config["order"]);
    p.refinement = static_cast<int>(config["refinement"]);
    p.iterations = static_cast<int>(config["iterations"]);
    p.mesh_resolution = static_cast<int>(config["mesh_resolution"]);
    p.surf_init_method = static_cast<SurfInitMethod>(static_cast<int>(config["surf_init_method"]));

    XmlRpc::XmlRpcValue surf_conf = config["fit_surface_parameters"];
    p.surface_params.interior_smoothness = static_cast<double>(surf_conf["interior_smoothness"]);
    p.surface_params.interior_weight = static_cast<double>(surf_conf["interior_weight"]);
    p.surface_params.boundary_smoothness = static_cast<double>(surf_conf["boundary_smoothness"]);
    p.surface_params.boundary_weight = static_cast<double>(surf_conf["boundary_weight"]);

    p.clip_boundary_curve = static_cast<bool>(config["clip_boundary_curve"]);
    if (!p.clip_boundary_curve)
    {
      return true;
    }

    p.boundary_fit_order = static_cast<int>(config["boundary_fit_order"]);
    XmlRpc::XmlRpcValue boundary_conf = config["boundary_curve_parameters"];
    p.boundary_startCPs = static_cast<int>(boundary_conf["startCPs"]);
    p.boundary_clipping_required = static_cast<bool>(boundary_conf["clipping_required"]);
    pcl::on_nurbs::FittingCurve2dAPDM::FitParameter& bp = p.boundary_curve_params;
    bp.addCPsAccuracy = static_cast<double>(boundary_conf["addCPsAccuracy"]);
    bp.addCPsIteration = static_cast<int>(boundary_conf["addCPsIteration"]);
    bp.maxCPs = static_cast<int>(boundary_conf["maxCPs"]);
    bp.accuracy = static_cast<double>(boundary_conf["accuracy"]);
    bp.iterations = static_cast<int>(boundary_conf["iterations"]);
    bp.param.closest_point_resolution = static_cast<double>(boundary_conf["closest_point_resolution"]);
    bp.param.closest_point_weight = static_cast<double>(boundary_conf["closest_point_weight"]);
    bp.param.closest_point_sigma2 = static_cast<double>(boundary_conf["closest_point_sigma2"]);
    bp.param.interior_sigma2 = static_cast<double>(boundary_conf["interior_sigma2"]);
    bp.param.smooth_concavity = static_cast<double>(boundary_conf["smooth_concavity"]);
    bp.param.smoothness = static_cast<double>(boundary_conf["smoothness"]);
  }
  catch (XmlRpc::XmlRpcException& e)
  {
    CONSOLE_BRIDGE_logError("Failed configure %s filter, error: %s", getName().c_str(), e.getMessage().c_str());
    return false;
  }
  return true;
}

bool BSplineReconstruction::filter(const pcl::PolygonMesh& mesh_in, pcl::PolygonMesh& mesh_out)
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
      CONSOLE_BRIDGE_logError("The surface initialization method %i is not yet implemented",
                              static_cast<int>(parameters_.surf_init_method));
      return false;
    default:
      CONSOLE_BRIDGE_logError("The surface initialization method %i was not recognized",
                              static_cast<int>(parameters_.surf_init_method));
      return false;
  }

  // fitting surface
  pcl::on_nurbs::FittingSurface fit(&nurbs_data, nurbs_surface);
  fit.setQuiet(!parameters_.verbosity_on);  // enable/disable debug output
  pcl::on_nurbs::FittingSurface::Parameter surf_params = parameters_.surface_params;
  for (int i = 0; i < parameters_.refinement; i++)
  {
    CONSOLE_BRIDGE_logDebug("Refinement attempt %i", i + 1);
    fit.refine(0);
    fit.refine(1);
    fit.assemble(surf_params);
    fit.solve();
  }

  // improving fit
  for (unsigned i = 0; i < parameters_.iterations; i++)
  {
    CONSOLE_BRIDGE_logDebug("Improvement attempt %i", i + 1);
    fit.assemble(surf_params);
    fit.solve();
  }

  if (!fit.m_nurbs.IsValid())
  {
    CONSOLE_BRIDGE_logError("Surface fitting failed");
    return false;
  }

  // fit B-spline boundary curve
  std::shared_ptr<pcl::on_nurbs::FittingCurve2dASDM> curve_fit = nullptr;
  if (parameters_.clip_boundary_curve)
  {
    // initialisation (circular)
    CONSOLE_BRIDGE_logDebug("Boundary Curve fitting ...");
    pcl::on_nurbs::NurbsDataCurve2d curve_data;
    curve_data.interior = nurbs_data.interior_param;
    curve_data.interior_weight_function.push_back(true);
    ON_NurbsCurve curve_nurbs = pcl::on_nurbs::FittingCurve2dAPDM::initNurbsCurve2D(
        parameters_.boundary_fit_order, curve_data.interior, parameters_.boundary_startCPs);

    // curve fitting
    curve_fit = std::make_shared<pcl::on_nurbs::FittingCurve2dASDM>(&curve_data, curve_nurbs);
    curve_fit->setQuiet(!parameters_.verbosity_on);  // enable/disable debug output
    curve_fit->fitting(parameters_.boundary_curve_params);

    if (!curve_fit->m_nurbs.IsValid())
    {
      if (parameters_.boundary_clipping_required)
      {
        CONSOLE_BRIDGE_logError("Failed to fit boundary curve");
        return false;
      }
      else
      {
        CONSOLE_BRIDGE_logWarn("Failed to fit boundary curve but returning unclipped surface");
      }
    }
  }

  if (curve_fit && curve_fit->m_nurbs.IsValid())
  {
    pcl::on_nurbs::Triangulation::convertTrimmedSurface2PolygonMesh(
        fit.m_nurbs, curve_fit->m_nurbs, mesh_out, parameters_.mesh_resolution);
  }
  else
  {
    pcl::on_nurbs::Triangulation::convertSurface2PolygonMesh(fit.m_nurbs, mesh_out, parameters_.mesh_resolution);
  }
  return true;
}

std::string BSplineReconstruction::getName() const { return utils::getClassName<decltype(*this)>(); }

} /* namespace mesh */
} /* namespace noether_filtering */
