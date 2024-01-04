/**
 * @brief This file was designed to be used with CMake CheckCXXSourceCompiles to see if PCL was compiled with OpenNurbs
 * support
 * @author Matthew Powelson
 * @file check_pcl_nurbs.cpp
 * @date June 18, 2020
 * @copyright Copyright (c) 2020, Southwest Research Institute
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
#include <pcl/surface/on_nurbs/triangulation.h>
#include <pcl/surface/on_nurbs/fitting_surface_tdm.h>
#include <pcl/surface/on_nurbs/fitting_curve_2d_asdm.h>

int main(int argc, char** argv)
{
  pcl::on_nurbs::FittingSurface::Parameter surface_params;
  pcl::on_nurbs::FittingCurve2dAPDM::FitParameter boundary_curve_params;
}
