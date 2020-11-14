/*
 * window_sinc_smoothing.h
 *
 *  Created on: Dec 6, 2019
 *      Author: jrgnicho
 */

#ifndef SRC_MESH_WINDOW_SINC_SMOOTHING_H_
#define SRC_MESH_WINDOW_SINC_SMOOTHING_H_

#include "noether_filtering/mesh/mesh_filter_base.h"

namespace noether_filtering
{
namespace mesh
{
/**
 * @class noether_filtering:;mesh::WindowedSincSmoothing
 * @details uses the vtkWindowedSincPolyDataFilter to smooth out the mesh, details of the implementation
 * and the configuration parameters can be found here
 * https://vtk.org/doc/nightly/html/classvtkWindowedSincPolyDataFilter.html
 */
class WindowedSincSmoothing : public MeshFilterBase
{
public:
  struct Config
  {
    std::size_t num_iter = 100;
    bool enable_boundary_smoothing = true;      /**@brief Set to true to enable */
    bool enable_feature_edge_smoothing = false; /**@brief Set to true to enable */
    bool enable_non_manifold_smoothing = true;  /**@brief Set to true to enable */
    bool enable_normalize_coordinates = true;   /**@brief Set to true to enable */
    double feature_angle = 10.0;                /**@brief degrees, only applicable when feature_edge_smoothing = true */
    double edge_angle = 150.0;                  /**@brief degrees, only applicable when feature_edge_smoothing = true */
    double pass_band = 0.01; /**@brief PassBand for the windowed sinc filter, see explanation in reference link */
  };

  WindowedSincSmoothing();
  virtual ~WindowedSincSmoothing();

  /**
   * @details Loads a configuration from yaml of the following form:
   *
   * num_iter: 100
   * enable_boundary_smoothing: true
   * enable_feature_edge_smoothing: false
   * enable_non_manifold_smoothing: true
   * enable_normalize_coordinates: true
   * feature_angle: 10.0
   * edge_angle: 150.0
   * pass_band: 0.01
   *
   * @param config
   * @return
   */
  bool configure(XmlRpc::XmlRpcValue config) override final;

  /**
   * @brief applies the filtering method
   * @param mesh_in   Input mesh
   * @param mesh_out  Output mesh
   * @return True on success, false otherwise
   */
  bool filter(const pcl::PolygonMesh& mesh_in, pcl::PolygonMesh& mesh_out) override final;

  /**
   * @brief returns the type name of the filter
   * @return  A string containing the filter type name
   */
  std::string getName() const override final;

private:
  Config config_;
};

} /* namespace mesh */
} /* namespace noether_filtering */

#endif /* SRC_MESH_WINDOW_SINC_SMOOTHING_H_ */
