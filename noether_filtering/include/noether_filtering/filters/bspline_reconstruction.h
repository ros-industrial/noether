/*
 * bspline_reconstruction.h
 *
 *  Created on: Oct 10, 2019
 *      Author: jrgnicho
 */

#ifndef INCLUDE_NOETHER_FILTERING_FILTERS_BSPLINE_RECONSTRUCTION_H_
#define INCLUDE_NOETHER_FILTERING_FILTERS_BSPLINE_RECONSTRUCTION_H_

#include <pcl/surface/on_nurbs/fitting_surface_tdm.h>
#include <pcl/surface/on_nurbs/fitting_curve_2d_asdm.h>
#include "noether_filtering/mesh_filter_base.h"

namespace noether_filtering
{
namespace filters
{

/**
 * @class noether_filtering::filters::BSplineReconstruction
 * @details: Smoothes a mesh by fitting a bspline surface to it.  The details of this implementation'
 * can be in found in http://pointclouds.org/documentation/tutorials/bspline_fitting.php
 */
class BSplineReconstruction : public MeshFilterBase
{
public:

  enum class SurfInitMethod: int
  {
    PCA = 1,
    PCA_BB,
    CUSTOM_PLANE, //  not implemented yet but will use pcl::on_nurbs::FittingSurface::initNurbs4Corners
  };

  struct Parameters
  {
    int order = 3;                    /** @brief is the polynomial order of the B-spline surface. */
    int refinement = 3;               /** @brief refinement is the number of refinement iterations, where for each iteration
                                                  control-points are inserted*/
    unsigned iterations = 1;          /** @brief is the number of iterations that are performed after refinement is completed */
    unsigned mesh_resolution = 50;    /** @brief  the number of vertices in each parametric direction, used for
                                                  triangulation of the B-spline surface.*/
    SurfInitMethod surf_init_method = SurfInitMethod::PCA_BB; /** @brief method for creating the initial surface to be fit */
    pcl::on_nurbs::FittingSurface::Parameter surface_params; /** @brief parameters to fit the surface**/

    bool clip_boundary_curve = true;  /** @brief whether to fit the boundary curve and clip every that extends past it*/
    int boundary_fit_order = 2;       /** @brief applicable only when clip_boundary_curve = true */
    pcl::on_nurbs::FittingCurve2dAPDM::FitParameter boundary_curve_params;
  };


  BSplineReconstruction();
  virtual ~BSplineReconstruction();

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
   * boundary_fit_order: 2 # applicable only when clip_boundary_curve: True
   * boundary_curve_parameters:
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
  bool configure(XmlRpc::XmlRpcValue config) final;
  bool filter(const pcl::PolygonMesh& mesh_in, pcl::PolygonMesh& mesh_out) final;
  std::string getName() final;

protected:
  Parameters parameters_;

};

} /* namespace filters */
} /* namespace noether_filtering */

#endif /* INCLUDE_NOETHER_FILTERING_FILTERS_BSPLINE_RECONSTRUCTION_H_ */
