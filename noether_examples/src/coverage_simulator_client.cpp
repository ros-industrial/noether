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
#include <string>

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
          if(x==grid_size_x)
          {
            points->InsertNextPoint(x, y, vtkMath::Random(0.0, 0.001));
          }
          else
          {
             points->InsertNextPoint(x+0.25, y, vtkMath::Random(0.0, 0.001));
          }
          break;
        case vtk_viewer::SINUSOIDAL_1D:
          if(x==grid_size_x)
          {
            points->InsertNextPoint(x, y, sin(double(y) / 2.0) + vtkMath::Random(0.0, 0.001));
          }
          else
          {
             points->InsertNextPoint(x+0.25, y, sin(double(y) / 2.0) + vtkMath::Random(0.0, 0.001));
          }
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

static std::string toLower(const std::string& in)
{
  std::string copy = in;
  std::transform(copy.begin(), copy.end(), copy.begin(), ::tolower);
  return copy;
}
//@brief This a helper function to read meshs from file
static bool readFile(std::string& file, std::vector<vtkSmartPointer<vtkPolyData> >& mesh)
{
  if (!file.empty())
  {
    // read data file
    vtkSmartPointer<vtkPolyData> data = vtkSmartPointer<vtkPolyData>::New();
    std::vector<char> buffer(file.size() + 1, '\0');
    char* str = buffer.data();
    strcpy(str, file.c_str());

    char* pch;
    pch = strtok(str, " ,.-");
    while (pch != NULL)
    {
      std::string extension(pch);
      if (extension == "pcd" || extension == "stl" || extension == "STL" || extension == "ply")
      {
        break;
      }
      pch = strtok(NULL, " ,.-");
    }

    std::string extension = std::string(pch);
    if (extension == "pcd")
    {
      vtk_viewer::loadPCDFile(file, data);
    }
    else if (extension == "STL" || extension == "stl")
    {
      data = vtk_viewer::readSTLFile(file);
    }
    else if (toLower(extension) == "ply")  // PCL polygon mesh
    {
      pcl::PolygonMesh pcl_mesh;
      vtk_viewer::loadPolygonMeshFromPLY(file, pcl_mesh);
      vtk_viewer::pclEncodeMeshAndNormals(pcl_mesh, data);
    }
    else
    {
      ROS_ERROR("Unrecognized extension: '%s'. Program supports 'pcd', 'stl', 'STL', 'ply'", extension.c_str());
      return false;
    }

    vtk_viewer::generateNormals(data);
    mesh.push_back(data);
    return true;
  }
  else
  {
    ROS_WARN_STREAM("'filename' parameter must be set to the path of a pcd or stl file");
    return false;
  }
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

  //......setup test meshes and paths
  pcl_msgs::PolygonMesh myMesh;
  vtkSmartPointer<vtkPoints> points2 = vtkSmartPointer<vtkPoints>::New();

  if(argc ==1)//if no file name given
  {
    //.......create control meshes
    for(int count = 0; count<1; count++)//start multimesh creation loop
    {
      vtkSmartPointer<vtkPoints> points;
      //if(count==0)
      //{
      //  points = vtk_viewer::createPlane(20,7,vtk_viewer::SINUSOIDAL_1D);
      //}
      if(count==0)
      {
        //points = vtk_viewer::createPlane(19,10,vtk_viewer::SINUSOIDAL_1D);
        points = vtk_viewer::createPlane(19,10,vtk_viewer::FLAT);
      }
      if(count==1)
      {
        points = createPlaneMod(19,10,vtk_viewer::SINUSOIDAL_1D);
        //points = createPlaneMod(19,10,vtk_viewer::FLAT);
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
      tool.pt_spacing = 5.0;
      tool.line_spacing = 1.0;
      tool.tool_offset = 1.0;
      tool.intersecting_plane_height = 0.5;
      tool.simulator_nearest_neighbors = 30;
      tool.min_hole_size = 0.1;
      tool.min_segment_size = 1;
      tool.raster_angle = 0;
      tool.raster_wrt_global_axes = 0;
      tool.simulator_tool_radius = 3.0;
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
  }//end no file name given

  else //.......read mesh from file
  {
    std::vector <vtkSmartPointer<vtkPolyData> > fileMesh;
    std::string fileLocation = argv[1];
    std::string fileType = ".stl";
    std::string curFile = " ";
    //.......create control meshes
    int numMeshs = 25;
    for(int count = 0; count<numMeshs; count++)//start multimesh creation loop
    {
      curFile ="";
      //ROS_INFO(curFile.c_str());
      curFile =fileLocation;
      curFile += std::to_string(count);
      curFile += fileType;
      readFile(curFile,fileMesh);
      vtkSmartPointer<vtkPolyData> cut = vtkSmartPointer<vtkPolyData>::New();
      cut = vtk_viewer::cutMesh(fileMesh[count], points2, false);
      pcl::PolygonMesh *test = new(pcl::PolygonMesh);
      pcl::VTKUtils::convertToPCL(cut, *test);
      pcl_conversions::fromPCL(*test, myMesh);
      if(count==4 ||count == 5||count==6|| count ==7 || count ==9 || count== 10 || (count >= 13&& count<24) )
      //if(count==22)
      {
        ROS_INFO("here");
        myMeshs.push_back(myMesh);
      }
      //........end create control meshes
      //........create robot path
      cut = vtk_viewer::cutMesh(fileMesh[count], points2, false);//possible redundant
      pcl::VTKUtils::convertToPCL(cut, *test);//posible redundant

      // Run tool path planner on mesh
      tool_path_planner::RasterToolPathPlanner planner;
      planner.setInputMesh(cut);

      tool_path_planner::ProcessTool tool;
      tool.pt_spacing = 50.0;
      tool.line_spacing = 50.0;
      tool.tool_offset = 0.0;
      tool.intersecting_plane_height = 0.5;
      tool.simulator_nearest_neighbors = 30;
      tool.min_hole_size = 0.1;
      tool.min_segment_size = 1;
      tool.raster_angle = 0;
      tool.raster_wrt_global_axes = 0;
      tool.simulator_tool_radius = 100.0;
      tool.simulator_tool_height = 20;
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
      if(count==4 ||count == 5|| count == 6|| count ==7 || count ==9 || count== 10 || (count >= 13&& count<24) )
      //if(count==22)
      {
        ROS_INFO("here path");
        rasterPath.paths = myPath;
        goal.path.push_back(rasterPath);
      }

      //........end robot path
    }//end multimesh creation loop

  }//end else
  //set goal values
  goal.input_mesh = myMeshs;
  ac.sendGoal(goal);
  ROS_INFO(" sent mesh and path generation");
  ROS_INFO("number of paths sent: %d ",goal.path.size());
  //wait for the action to return
  bool finished_before_timeout = ac.waitForResult();

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
