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

#pragma once

#include <noether_tpp/core/mesh_modifier.h>
#include <pcl/surface/on_nurbs/fitting_surface_tdm.h>
#include <pcl/surface/on_nurbs/fitting_curve_2d_asdm.h>

namespace noether
{
/**
 * @ingroup mesh_modifiers
 * @brief Smoothes a mesh by fitting a bspline surface to it
 * @details The details of this implementation can be in found in
 * @sa http://pointclouds.org/documentation/tutorials/bspline_fitting.php
 */
class BSplineReconstruction : public MeshModifier
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
    /** @brief print more info */
    bool verbosity_on = false;
    /** @brief is the polynomial order of the B-spline surface. */
    int order = 3;
    /** @brief refinement is the number of refinement iterations, where for each iteration control-points are inserted
     */
    int refinement = 3;
    /** @brief is the number of iterations that are performed after refinement is completed */
    unsigned iterations = 1;
    /** @brief  the number of vertices in each parametric direction, used for triangulation of the B-spline surface.*/
    unsigned mesh_resolution = 50;
    /** @brief method for creating the initial surface to be fit */
    SurfInitMethod surf_init_method = SurfInitMethod::PCA_BB;
    /** @brief parameters to fit the surface**/
    pcl::on_nurbs::FittingSurface::Parameter surface_params;
    /** @brief whether to fit the boundary curve and clip every that extends past it*/
    bool clip_boundary_curve = true;
    /** @brief applicable only when clip_boundary_curve = true */
    int boundary_fit_order = 2;
    /** @brief initial number of control points */
    int boundary_startCPs = 0;
    /** @brief if True then algorithm will fail if the boundary could not be clipped */
    bool boundary_clipping_required = false;
    pcl::on_nurbs::FittingCurve2dAPDM::FitParameter boundary_curve_params;
  };

  std::vector<pcl::PolygonMesh> modify(const pcl::PolygonMesh& mesh) const override;

protected:
  Parameters parameters_;
};

}  // namespace noether
