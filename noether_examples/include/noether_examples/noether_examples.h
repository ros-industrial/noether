/**
 * @file noether_examples.h
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

#ifndef INCLUDE_NOETHER_EXAMPLES_H
#define INCLUDE_NOETHER_EXAMPLES_H

#include <vtk_viewer/vtk_viewer.h>
#include <mesh_segmenter/mesh_segmenter.h>
#include <tool_path_planner/path_generator.h>
#include <path_sequence_planner/path_sequence_planner.h>

namespace noether
{
class Noether
{
public:
  Noether() = default;
  virtual ~Noether() = default;

  /**
   * @brief addMeshDisplay Add a display for a vector of meshes (each mesh will be colored differently)
   * @param meshes The vector to be displayed
   */
  void addMeshDisplay(std::vector<vtkSmartPointer<vtkPolyData> >& meshes);

  /**
   * @brief addPathDisplay  Add a display for a vector of tool paths
   * @param paths The input paths to add to the display
   * @param scale The size to scale and show the arrows at
   * @param show_path If true, will show the points in the paths with normal vectors
   * @param show_cutting_meshes If true, will show the cutting meshes used to calculate the path points
   * @param show_derivatives If true, will show the derivatives for each point in the path
   */
  void addPathDisplay(std::vector<tool_path_planner::ToolPaths>& paths,
                      double scale,
                      bool show_path = true,
                      bool show_cutting_meshes = false,
                      bool show_derivatives = false);

  /**
   * @brief addPathDisplay  Add a display for a vector of tool paths
   * @param paths The input paths to add to the display
   * @param scale The size to scale and show the arrows at
   * @param show_path If true, will show the points in the paths with normal vectors
   * @param show_cutting_meshes If true, will show the cutting meshes used to calculate the path points
   * @param show_derivatives If true, will show the derivatives for each point in the path
   */
  void addPathDisplay(tool_path_planner::ToolPaths& paths,
                      double scale,
                      bool show_path = true,
                      bool show_cutting_meshes = false,
                      bool show_derivatives = false);

  /** @brief visualizeDisplay Calls the VTK render function */
  void visualizeDisplay() { viewer_.renderDisplay(); }

  /**
   * @brief setLogDir Set the directory for saving polydata files to
   * @param dir The directory to save data to
   */
  void setLogDir(std::string dir) { viewer_.setLogDir(dir); }

  /**
   * @brief getLogDir Get the directory used for saving polydata files to
   * @return The directory currently used for saving data
   */
  std::string getLogDir() { return viewer_.getLogDir(); }

private:
  vtk_viewer::VTKViewer viewer_; /**< The VTK viewer for displaying all objects */
};
}  // namespace noether

#endif  // INCLUDE_NOETHER_EXAMPLES_H
