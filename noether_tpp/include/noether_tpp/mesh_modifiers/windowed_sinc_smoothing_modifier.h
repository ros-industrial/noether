/*
 * window_sinc_smoothing.h
 *
 *  Created on: Dec 6, 2019
 *      Author: jrgnicho
 */

#pragma once

#include <noether_tpp/core/mesh_modifier.h>

namespace noether
{
/**
 * @class noether_filtering:;mesh::WindowedSincSmoothing
 * @details uses the vtkWindowedSincPolyDataFilter to smooth out the mesh, details of the implementation
 * and the configuration parameters can be found here
 * https://vtk.org/doc/nightly/html/classvtkWindowedSincPolyDataFilter.html
 */
class WindowedSincSmoothing : public MeshModifier
{
public:
  struct Config
  {
    std::size_t num_iter = 100;
    /**@brief Set to true to enable */
    bool enable_boundary_smoothing = true;
    /**@brief Set to true to enable */
    bool enable_feature_edge_smoothing = false;
    /**@brief Set to true to enable */
    bool enable_non_manifold_smoothing = true;
    /**@brief Set to true to enable */
    bool enable_normalize_coordinates = true;
    /**@brief degrees, only applicable when feature_edge_smoothing = true */
    double feature_angle = 10.0;
    /**@brief degrees, only applicable when feature_edge_smoothing = true */
    double edge_angle = 150.0;
    /**@brief PassBand for the windowed sinc filter, see explanation in reference link */
    double pass_band = 0.01;
  };

  std::vector<pcl::PolygonMesh> modify(const pcl::PolygonMesh& mesh) const override;

protected:
  Config config_;
};

}  // namespace noether
