#include <ros/ros.h>
#include <ros/ros.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <vtk_viewer/vtk_utils.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <pcl/PolygonMesh.h>
#include <vtkPointData.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tool_path_planner/raster_tool_path_planner.h>

#include <tool_path_planner/utilities.h>
#include <mesh_segmenter/mesh_segmenter.h>
#include <vtkPolyDataNormals.h>
#include <vtkCellData.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <noether_msgs/ThickSimulatorAction.h>
#include <noether_msgs/ToolRasterPath.h>



//make modified meshes
vtkSmartPointer<vtkPoints> createPlaneMod(unsigned int grid_size_x, unsigned int grid_size_y, vtk_viewer::plane_type type)
{
  // Create points on an XY grid with a sinusoidal Z component
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();

  for (unsigned int x = grid_size_x; x < 10; x++)
  {
    for (unsigned int y = 0; y < grid_size_y; y++)
    {
      switch (type)
      {
        case vtk_viewer::FLAT:
          points->InsertNextPoint(x, y, vtkMath::Random(0.0, 0.001));
          break;
        case vtk_viewer::SINUSOIDAL_1D:
          points->InsertNextPoint(x, y, sin(double(y) / 2.0) + vtkMath::Random(0.0, 0.001));
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
  actionlib::SimpleActionClient<noether_msgs::ThickSimulatorAction> ac("noether_simulator", true);

  ROS_INFO("Waiting for action server to start.");
  // wait for the action server to start
  ac.waitForServer();

  ROS_INFO("Action server started, sending goal.");
  // send a goal to the action
  noether_msgs::ThickSimulatorGoal goal;

  //......setup test meshes and path
  pcl_msgs::PolygonMesh myMesh;
  std::vector <pcl_msgs::PolygonMesh> myMeshs;
  std::vector <geometry_msgs::PoseArray> myPath;

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
  for(int count = 0; count<3; count++)//start multimesh creation loop
  {
    vtkSmartPointer<vtkPoints> points;
    if(count==0)
    {
      points = vtk_viewer::createPlane(10,10,vtk_viewer::SINUSOIDAL_2D);
    }
    if(count==1)
    {
      points = vtk_viewer::createPlane(9,10,vtk_viewer::SINUSOIDAL_2D);
    }
    if(count==2)
    {
      points = createPlaneMod(8,10,vtk_viewer::SINUSOIDAL_2D);
    }

    vtkSmartPointer<vtkPolyData> data = vtkSmartPointer<vtkPolyData>::New();
    data = vtk_viewer::createMesh(points, 0.5, 5);

    vtk_viewer::generateNormals(data);

    vtkSmartPointer<vtkPolyData> cut = vtkSmartPointer<vtkPolyData>::New();
    cut = vtk_viewer::cutMesh(data, points2, false);
    pcl::PolygonMesh *test = new(pcl::PolygonMesh);
    pcl::VTKUtils::convertToPCL(cut, *test);
    pcl_conversions::fromPCL(*test, myMesh);
    if(count>0)
    {
      myMeshs.push_back(myMesh);
    }
    //........end create control meshes
    //........create robot path
    //if(count==0)
    //{
      //vtkSmartPointer<vtkPolyData> cut = vtkSmartPointer<vtkPolyData>::New();
      cut = vtk_viewer::cutMesh(data, points2, false);
      //pcl::PolygonMesh *test = new(pcl::PolygonMesh);
      pcl::VTKUtils::convertToPCL(cut, *test);

      // Run tool path planner on mesh
      tool_path_planner::RasterToolPathPlanner planner;
      planner.setInputMesh(cut);

      tool_path_planner::ProcessTool tool;
      tool.pt_spacing = 0.5;
      tool.line_spacing = .75;
      tool.tool_offset = 0.0; // currently unused
      tool.intersecting_plane_height = 0.15; // 0.5 works best, not sure if this should be included in the tool
      tool.nearest_neighbors = 30; // not sure if this should be a part of the tool
      tool.min_hole_size = 0.1;
      tool.min_segment_size = 1;
      tool.raster_angle = 0;
      tool.raster_wrt_global_axes = 0;
      tool.tool_radius = 1;
      tool.tool_height = 2;
      planner.setTool(tool);
      planner.setDebugMode(false);

      planner.computePaths();


      std::vector<tool_path_planner::ProcessPath> paths = planner.getPaths();
      myPath = tool_path_planner::toPosesMsgs(paths);

      noether_msgs::ToolRasterPath rasterPath;
      rasterPath.paths = myPath;
      goal.path.push_back(rasterPath);

    //}
    //........end robot path
  }//end multimesh creation loop

  //set goal values
  goal.input_mesh = myMeshs;
  //noether_msgs::ToolRasterPath rasterPath;
  //rasterPath.paths = myPath;
  //goal.path.push_back(rasterPath);
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
    ROS_INFO("part %d, is %d",i+1,ac.getResult()->painted[i]);
  }
  ROS_INFO("number of paths required is: %d",ac.getResult()->requiredPath.size());
  //exit
  return 0;
}
