#ifndef NOETHER_SIMULATOR_H
#define NOETHER_SIMULATOR_H

/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2016, Southwest Research Institute
 *
 * file noether_simulator.h
 * All rights reserved.
 * copyright Copyright (c) 2019, Southwest Research Institute
 *
 * License
 * Software License Agreement (Apache License)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <path_sequence_planner/simple_path_sequence_planner.h>
#include <tool_path_planner/tool_path_planner_base.h>
#include <tool_path_planner/raster_tool_path_planner.h>
#include <vtk_viewer/vtk_utils.h>
#include <vtk_viewer/vtk_viewer.h>
#include <vtkIdTypeArray.h>
#include <vtkDoubleArray.h>
#include <vtkPointData.h>
#include <vtkMatrix4x4.h>

namespace noether_simulator
{

class NoetherSimulator
{
public:

  NoetherSimulator();
  ~NoetherSimulator(){}

   /**
   * @brief getSimulatedPoints returns the simulated_points_ from run simulation
   */
  vtkSmartPointer<vtkPolyData>  getSimulatedPoints();
  /**
   * @brief setDebugModeOn Turn on debug mode to visualize every step of the path planning process
   * @param debug Turns on debug if true, turns off debug if false
   */
  void setDebugMode(bool debug){debug_on_ = debug;}

  /**
   * @brief setInputMesh Sets the input mesh to generate paths
   * @param mesh The input mesh to be operated on
   */
  void setInputMesh(vtkSmartPointer<vtkPolyData> mesh){input_mesh_ = mesh;}

  /**
   * @brief setInputMesh Sets the input mesh to generate paths
   * @param mesh The input mesh to be operated on
   */
  void setInputPaths(std::vector<tool_path_planner::ProcessPath> paths){input_paths_ = paths;}

  /**
   * @brief setTool Sets the tool parameters used during path generation
   * @param tool The tool object with all of the parameters necessary for path generation
   */
  void setTool(tool_path_planner::ProcessTool tool){tool_ = tool;}

  void runSimulation();

  void setDisplaySigma(double sigma){scalar_sigma_ = sigma;}

  void setBaseProcessRate(double rate){process_base_rate_ = rate;}

  void displayResults();

private:

  double process_base_rate_; /**< For a given process (i.e. painting), the base material processing rate (in this case, the amount of paint flow at the tip) */
  double scalar_sigma_; /**< number of sigmas for displaying final simulation coloring, higher sigma will show a "smoother" result */
  bool debug_on_;  /**< Turns on/off the debug display which views the simulation output one step at a time */
  vtk_viewer::VTKViewer debug_viewer_;  /**< The vtk viewer for displaying debug output */

  vtkSmartPointer<vtkPolyData> simulation_points_; /**< The simulated points with resulting process data */
  vtkSmartPointer<vtkPolyData> input_mesh_; /**< input mesh to operate on */
  std::vector<tool_path_planner::ProcessPath> input_paths_;  /**< input paths to simulate process on */
  tool_path_planner::ProcessTool tool_; /**< The tool parameters which defines how to generate the tool paths (spacing, offset, etc.) */

  vtkSmartPointer<vtkMatrix4x4> createMatrix(double pt[3], double norm[3], double derv[3]);

  double integral(double pt[3]);
  double calculateIntegration(vtkSmartPointer<vtkPoints> points);
};

}

#endif // NOETHER_SIMULATOR_H

