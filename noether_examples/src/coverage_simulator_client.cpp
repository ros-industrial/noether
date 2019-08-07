/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2016, Southwest Research Institute
 *
 * file thickness_simulator_client.cpp
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

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <pcl/PolygonMesh.h>
#include <pcl_conversions/pcl_conversions.h>

#include <vtkPolyDataNormals.h>
#include <vtkCellData.h>
#include <vtkPointData.h>
#include <vtk_viewer/vtk_utils.h>

#include <tool_path_planner/raster_tool_path_planner.h>
#include <tool_path_planner/utilities.h>
#include <mesh_segmenter/mesh_segmenter.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include <noether_msgs/SimulateCoverageAction.h>
#include <noether_msgs/ToolRasterPath.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

/*This example shows how the thickness_simulator_node.cpp works.
 * First, it creates 3 meshes and three paths.
 * Then, it passes these meshes and paths to the thickness_simulator_node.
 * The thickness_simulator_node runs the first path across each mesh.
 * If the mesh is sufficently covered the bool corasponding to that mesh in the coverage[]
 * is set to true, the mesh is excluded from future iterations, and the path is added to the reuqiredPaths[] unless,
 * it has already been added.
 * Once all the meshes have been evaluated using the first path, the simulator moves on to the second path.
 * The simulator evaluates the remaining meshes with the second path and so on until all meshes have the coraspoinding
 * bool value in coverage[] set to true, or all paths have been tried.
 *
 *usage:
 * roslaunch noether_examples coverage_sim.launch
 *
 * //in a seperate terminal:
 * rosrun noether_examples coverage_simulator_client
*/

//@brief This a helper function to create test meshes
vtkSmartPointer<vtkPoints> createPlaneMod(unsigned int grid_size_x, unsigned int grid_size_y, vtk_viewer::plane_type type)
{
  // Create points on an XY grid with a sinusoidal Z component
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();

  for (unsigned int x = grid_size_x; x < 21; x++)
  {
    for (unsigned int y = 0; y < grid_size_y; y++)
    {
      switch (type)
      {
        case vtk_viewer::FLAT:
          points->InsertNextPoint(x, y, vtkMath::Random(0.0, 0.001));
          break;
        case vtk_viewer::SINUSOIDAL_1D:
          if(x==grid_size_x)
          {
            points->InsertNextPoint(x, y, sin(double(y) / 2.0) + vtkMath::Random(0.0, 0.001));
          }
          else
          {
             points->InsertNextPoint(x+0.25, y, sin(double(y) / 2.0) + vtkMath::Random(0.0, 0.001));          }
          break;
        case vtk_viewer::SINUSOIDAL_2D:
        if(x==grid_size_x)
        {
           points->InsertNextPoint(x, y, 1.0 * cos(double(x) / 2.0) - 1.0 * sin(double(y) / 2.0) + vtkMath::Random(0.0, 0.001));
        }
        else
        {
           points->InsertNextPoint(x+0.25, y, 1.0 * cos(double(x) / 2.0) - 1.0 * sin(double(y) / 2.0) + vtkMath::Random(0.0, 0.001));
        }
        break;
      }
    }
  }

  // Add the grid points to a polydata object
  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
  polydata->SetPoints(points);

  return points;
}

int main (int argc, char **argv)
{  
  ros::init(argc, argv, "test_noether_simulator");

  // create the action client
  // true causes the client to spin its own thread
  actionlib::SimpleActionClient<noether_msgs::SimulateCoverageAction> ac("noether_simulator", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer();

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  noether_msgs::SimulateCoverageGoal goal;

  std::vector <pcl_msgs::PolygonMesh> myMeshs;
  std::vector <geometry_msgs::PoseArray> myPath;

    //......setup test meshes and path
    pcl_msgs::PolygonMesh myMesh;
    vtkSmartPointer<vtkPoints> points2 = vtkSmartPointer<vtkPoints>::New();
    double pt1[3] = {3.0, 3.0, 0.0};
    double pt2[3] = {4.0, 2.0, 0.0};
    double pt3[3] = {5.0, 3.0, 0.0};
    double pt4[3] = {4.0, 4.0, 0.0};

    points2->InsertNextPoint(pt1);
    points2->InsertNextPoint(pt2);
    points2->InsertNextPoint(pt3);
    points2->InsertNextPoint(pt4);
    //......end setup test meshes and path

    //.......create control meshes
    for(int count = 0; count<2; count++)//start multimesh creation loop
    {
      vtkSmartPointer<vtkPoints> points;
      if(count==0)
      {
        points = vtk_viewer::createPlane(19,10,vtk_viewer::SINUSOIDAL_1D);
      }
      if(count==1)
      {
        points = createPlaneMod(19,10,vtk_viewer::SINUSOIDAL_1D);
      }

      vtkSmartPointer<vtkPolyData> data = vtkSmartPointer<vtkPolyData>::New();
      data = vtk_viewer::createMesh(points, 0.5, 5);

      vtk_viewer::generateNormals(data);

      vtkSmartPointer<vtkPolyData> cut = vtkSmartPointer<vtkPolyData>::New();
      cut = vtk_viewer::cutMesh(data, points2, false);
      pcl::PolygonMesh *test = new(pcl::PolygonMesh);
      pcl::VTKUtils::convertToPCL(cut, *test);
      pcl_conversions::fromPCL(*test, myMesh);

      myMeshs.push_back(myMesh);
      //........end create control meshes
      //........create robot path
      cut = vtk_viewer::cutMesh(data, points2, false);
      pcl::VTKUtils::convertToPCL(cut, *test);

      // Run tool path planner on mesh
      tool_path_planner::RasterToolPathPlanner planner;
      planner.setInputMesh(cut);

      tool_path_planner::ProcessTool tool;
      tool.pt_spacing = 0.5;
      tool.line_spacing = 0.75;
      tool.tool_offset = 0.0;
      tool.intersecting_plane_height = 0.15;
      tool.simulator_nearest_neighbors = 30;
      tool.min_hole_size = 0.1;
      tool.min_segment_size = 1;
      tool.raster_angle = 0;
      tool.raster_wrt_global_axes = 0;
      tool.simulator_tool_radius = 10.0;
      tool.simulator_tool_height = 1;
      planner.setTool(tool);
      planner.setDebugMode(false);
      planner.computePaths();
      std::vector<tool_path_planner::ProcessPath> paths = planner.getPaths();
      //display path
      vtk_viewer::VTKViewer viz;
      std::vector<float> color(3);

      double scale = 1.0;

      // Display mesh results
      color[0] = 0.9;
      color[1] = 0.9;
      color[2] = 0.9;
      viz.addPolyDataDisplay(cut, color);

      for(int i = 0; i < paths.size(); ++i)
      {
      color[0] = 0.2;
      color[1] = 0.9;
      color[2] = 0.2;
      viz.addPolyNormalsDisplay(paths[i].line, color, scale);

      color[0] = 0.9;
      color[1] = 0.9;
      color[2] = 0.2;
      viz.addPolyNormalsDisplay(paths[i].derivatives, color, scale);
      }
      viz.renderDisplay();
      //end display path

      myPath = tool_path_planner::toPosesMsgs(paths);
      noether_msgs::ToolRasterPath rasterPath;
      rasterPath.paths = myPath;
      goal.path.push_back(rasterPath);

      //........end robot path
    }//end multimesh creation loop

  //set goal values
  goal.input_mesh = myMeshs;
  ac.sendGoal(goal);
  ROS_INFO(" sent mesh and path generation");
  ROS_INFO("number of paths sent: %d ",goal.path.size());
  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult(ros::Duration(300.0));

  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = ac.getState();
    ROS_INFO("Action finished: %s",state.toString().c_str());
  }
  else
    ROS_INFO("Action did not finish before the time out.");

  for(int i=0; i<goal.input_mesh.size(); i++)
  {
    ROS_INFO("part %d, is %d",i+1,ac.getResult()->coverage[i]);
  }
  ROS_INFO("number of paths required is: %d",ac.getResult()->requiredPath.size());
  //exit
  return 0;
}
