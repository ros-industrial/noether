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

#ifndef INCLUDE_NOETHER_FILTERING_MESH_FILTERS_BSPLINE_RECONSTRUCTION_H_
#define INCLUDE_NOETHER_FILTERING_MESH_FILTERS_BSPLINE_RECONSTRUCTION_H_

#include "noether_filtering/mesh/mesh_filter_base.h"
#include <pcl/surface/on_nurbs/fitting_surface_tdm.h>
#include <pcl/surface/on_nurbs/fitting_curve_2d_asdm.h>

namespace noether_filtering
{
namespace mesh
{
/**
 * @class noether_filtering::filters::BSplineReconstruction
 * @details: Smoothes a mesh by fitting a bspline surface to it.  The details of this implementation'
 * can be in found in http://pointclouds.org/documentation/tutorials/bspline_fitting.php
 */
class BSplineReconstruction : public MeshFilterBase
{
public:
  enum class SurfInitMethod : int
  {
    PCA = 1,
    PCA_BB,
    CUSTOM_PLANE,  //  not implemented yet but will use pcl::on_nurbs::FittingSurface::initNurbs4Corners
  };

  struct Parameters
  {
    bool verbosity_on = false; /** @brief print more info */
    int order = 3;             /** @brief is the polynomial order of the B-spline surface. */
    int refinement = 3;        /** @brief refinement is the number of refinement iterations, where for each iteration
                                           control-points are inserted*/
    unsigned iterations = 1; /** @brief is the number of iterations that are performed after refinement is completed */
    unsigned mesh_resolution = 50; /** @brief  the number of vertices in each parametric direction, used for
                                               triangulation of the B-spline surface.*/
    SurfInitMethod surf_init_method =
        SurfInitMethod::PCA_BB; /** @brief method for creating the initial surface to be fit */
    pcl::on_nurbs::FittingSurface::Parameter surface_params; /** @brief parameters to fit the surface**/

    bool clip_boundary_curve = true; /** @brief whether to fit the boundary curve and clip every that extends past it*/
    int boundary_fit_order = 2;      /** @brief applicable only when clip_boundary_curve = true */
    int boundary_startCPs = 0;       /** @brief initial number of control points */
    bool boundary_clipping_required =
        false; /** @brief if True then algorithm will fail if the boundary could not be clipped */
    pcl::on_nurbs::FittingCurve2dAPDM::FitParameter boundary_curve_params;
  };

  using FilterBase<pcl::PolygonMesh>::FilterBase;

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
  bool configure(XmlRpc::XmlRpcValue config) override final;
  bool filter(const pcl::PolygonMesh& mesh_in, pcl::PolygonMesh& mesh_out) override final;
  std::string getName() const override final;

protected:
  Parameters parameters_;
};

} /* namespace mesh */
} /* namespace noether_filtering */

#endif /* INCLUDE_NOETHER_FILTERING_MESH_FILTERS_BSPLINE_RECONSTRUCTION_H_ */
