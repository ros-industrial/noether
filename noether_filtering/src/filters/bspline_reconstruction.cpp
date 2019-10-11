/*
 * bspline_reconstruction.cpp
 *
 *  Created on: Oct 10, 2019
 *      Author: jrgnicho
 */

#include "noether_filtering/filters/bspline_reconstruction.h"
#include "noether_filtering/utils.h"
#include <XmlRpcException.h>
#include <pluginlib/class_list_macros.h>

PLUGINLIB_EXPORT_CLASS(noether_filtering::filters::BSplineReconstruction, noether_filtering:: MeshFilterBase)

namespace noether_filtering
{
namespace filters
{
BSplineReconstruction::BSplineReconstruction()
{

}

BSplineReconstruction::~BSplineReconstruction()
{

}

/** @details
 * Loads the configuration from a yaml object with the following structure:
 * order: 3
 * refinement: 3
 * iterations: 1
 * mesh_resolution: 50
 * surf_init_method: 2
 * fit_surface_parameters:
 *  interior_smoothness: 0.2
 *  interior_weight: 1.0
 *  boundary_smoothness: 0.2
 *  boundary_weight: 0.0
 * clip_boundary_curve: True
 * boundary_fit_order: 2      # applicable only when clip_boundary_curve: True
 * boundary_curve_parameters: # applicable only when clip_boundary_curve: True
 *  addCPsAccuracy: 5e-2
 *  addCPsIteration: 3
 *  maxCPs: 200
 *  accuracy: 1e-3
 *  iterations: 100
 *  closest_point_resolution: 0
 *  closest_point_weight: 1.0
 *  closest_point_sigma2: 0.1
 *  interior_sigma2: 0.00001
 *  smooth_concavity: 1.0
 *  smoothness: 1.0
 *
 * @param config The configuration
 * @return True on success, false otherwise
 */
bool BSplineReconstruction::configure(XmlRpc::XmlRpcValue config)
{
  Parameters& p = parameters_;
  try
  {
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
    if(!p.clip_boundary_curve)
    {
      return true;
    }

    p.boundary_fit_order = static_cast<int>(surf_conf["boundary_fit_order"]);
    XmlRpc::XmlRpcValue boundary_conf = config["boundary_curve_parameters"];
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
  catch(XmlRpc::XmlRpcException& e)
  {
    CONSOLE_BRIDGE_logError("Failed configure %s filter, error: %s",getName().c_str(),
                            e.getMessage().c_str());
    return false;
  }
  return true;
}

bool BSplineReconstruction::filter(const pcl::PolygonMesh& mesh_in, pcl::PolygonMesh& mesh_out)
{
}

std::string BSplineReconstruction::getName()
{
  return std::move(utils::getClassName<decltype(*this)>());
}

} /* namespace filters */
} /* namespace noether_filtering */
