#include "ros/ros.h"
#include <actionlib/server/simple_action_server.h>
#include <ros/service_server.h>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>

#include <vtk_viewer/vtk_utils.h>
#include <vtkPointData.h>
#include <Eigen/Core>

#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <pcl_conversions/pcl_conversions.h>
#include <dynamic_reconfigure/server.h>

#include "noether_simulator/noether_simulator.h"
#include "noether_simulator/NoetherSimulatorConfig.h"
#include "noether_msgs/ThickSimulatorAction.h"

#include <tool_path_planner/tool_path_planner_base.h>
#include <tool_path_planner/raster_tool_path_planner.h>
#include <pcl/kdtree/kdtree_flann.h>


namespace noether_simulator {
class ProcessSimulatorNode{
private:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<noether_msgs::ThickSimulatorAction> simulation_service_;

  double tool_height_;
  double process_rate_;
  double display_sigma_;
  double tool_radius_;

  void updateParameters(noether_simulator::NoetherSimulatorConfig& config, uint32_t level);
  dynamic_reconfigure::Server<noether_simulator::NoetherSimulatorConfig> reconfigurer_;
  boost::mutex param_lock_;
  //tool_path_planner::ProcessTool tool_;
  double vect_[3], center_[3];
  bool debug_on_;
  std::string log_directory_;
  noether_msgs::ThickSimulatorFeedback feedback_;
  noether_msgs::ThickSimulatorResult result_;

  tool_path_planner::ProcessPath convertPoseArraytoVTK(geometry_msgs::PoseArray array)
  {
    tool_path_planner::ProcessPath path;

    vtkSmartPointer<vtkPolyData> line = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPolyData> derivatives_ = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPoints> pts = vtkSmartPointer<vtkPoints>::New();

    // set the point positions
    for(int ii = 0; ii < array.poses.size(); ii++)
    {
      geometry_msgs::Pose pose = array.poses[ii];

      double pt[3];
      pt[0] = (double)pose.position.x;
      pt[1] = (double)pose.position.y;
      pt[2] = (double)pose.position.z;

      pts->InsertNextPoint(pt);
    }

    line->SetPoints(pts);
    derivatives_->SetPoints(pts);

    // calculate point normals
    vtkSmartPointer<vtkDoubleArray> normals = vtkSmartPointer<vtkDoubleArray>::New();
    vtkSmartPointer<vtkDoubleArray> deriv_normals = vtkSmartPointer<vtkDoubleArray>::New();
    normals->SetNumberOfComponents(3);
    deriv_normals->SetNumberOfComponents(3);

    for(int ii = 0; ii < line->GetPoints()->GetNumberOfPoints(); ii++)
    {
      double pt[3];
      geometry_msgs::Quaternion q = array.poses[ii].orientation;

      //calculate z-axis from quaternion
      double x = (2*q.x*q.z + 2*q.y*q.w);
      double y = (2*q.y*q.z - 2*q.x*q.w);
      double z = (1 - 2*q.x*q.x - 2*q.y*q.y);
      double n[3] = {x, y, z};
      normals->InsertNextTypedTuple(n);

      // calculate y-axis from quaternion
      x = (2*q.x*q.y - 2*q.z*q.w);
      y = (1 - 2*q.x*q.x - 2*q.z*q.z);
      z = (1 - 2*q.y*q.z + 2*q.x*q.w);
      double m[3] = {x, y, z};
      deriv_normals->InsertNextTypedTuple(m);
    }
    line->GetPointData()->SetNormals(normals);
    path.line = line;

    derivatives_->GetPointData()->SetNormals(deriv_normals);
    path.derivatives = derivatives_;

    vtkSmartPointer<vtkPolyData> inter_ = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkParametricSpline> spline_ = vtkSmartPointer<vtkParametricSpline>::New();
    path.intersection_plane = inter_;
    path.spline = spline_;

    return path;
  }

public:

  ProcessSimulatorNode(std::string name) :
    simulation_service_(nh_, name, boost::bind(&ProcessSimulatorNode::executeCB,this, _1), false),
    tool_height_(2),
    process_rate_(0),//not used in utest
    display_sigma_(0),//not used in utest
    tool_radius_(1)
  {
    simulation_service_.start();
  }
  void executeCB(const noether_msgs::ThickSimulatorGoalConstPtr &goal)
  {
    ros::Rate r(0.05);
    bool success = true;
    feedback_.percent = 0.0;
    ROS_INFO("simulator running");
    int num_meshes= goal->input_mesh.size();
    int num_paths = goal->path.size();
    noether_simulator::NoetherSimulator sim;

    //add tool
    tool_path_planner::ProcessTool tool;
    tool.pt_spacing = 0.5;
    tool.line_spacing = 0.75;
    tool.tool_offset = 0.0; // currently unused
    tool.intersecting_plane_height = 0.15; // 0.5 works best, not sure if this should be included in the tool
    tool.nearest_neighbors = 30; // not sure if this should be a part of the tool
    tool.min_hole_size = 0.1;
    tool.min_segment_size = 1;
    tool.raster_angle = 0;
    tool.raster_wrt_global_axes = 0;
    tool.tool_radius = 1;
    tool.tool_height = 2;
    sim.setTool(tool);

    for(int i=0; i<num_meshes;i++)
    {
      result_.painted.push_back(false);
    }
    //start multiple paths
    for (int curPathNum=0;curPathNum<goal->path.size();curPathNum++)
    {
      bool added = false;
      //convert robot path for simulator
      std::vector<geometry_msgs::PoseArray> robot_paths_in = goal->path[curPathNum].paths;
      std::vector<tool_path_planner::ProcessPath> robot_paths_arg;
      int number_paths = robot_paths_in.size();

      for(int ii = 0; ii < number_paths; ii++)
      {
        robot_paths_arg.push_back(convertPoseArraytoVTK(robot_paths_in[ii]));
      }

      sim.setInputPaths(robot_paths_arg);//add path to simulator

      //run simulator for each mesh
      for(int i=0; i <num_meshes; i++)
      {
        if(simulation_service_.isPreemptRequested() || !ros::ok())
        {
          simulation_service_.setPreempted();
          success = false;
          break;
        }
        if(result_.painted[i]==false)//if not yet painted
        {

          //convert mesh for simulator
          pcl::PolygonMesh surface_mesh;
          pcl_conversions::toPCL(goal->input_mesh[i], surface_mesh);
          vtkSmartPointer<vtkPolyData> surface_mesh_vtk = vtkSmartPointer<vtkPolyData>::New();
          pcl::VTKUtils::convertToVTK(surface_mesh, surface_mesh_vtk);

          sim.setInputMesh(surface_mesh_vtk);//add mesh to simulator
          sim.runSimulation();//display painted parts

          //deterimine if painted enough
          vtkSmartPointer<vtkPolyData> processedPoints = vtkSmartPointer<vtkPolyData>::New();
          processedPoints = sim.getSimulatedPoints();//get a copy of simulated points to operate on
          vtkIdType length = processedPoints->GetNumberOfPoints();
          double intensity[3];
          int missed = 0;
          double minPaintThreshold = 5;
          double maxPassableNonpainted = 0.1;

          //******TODO*******//
          //  replace with FLANN
          for(vtkIdType a = 0; a < length; a++)//iterate through points to check if painted
          {
            intensity[0] = processedPoints->GetPointData()->GetScalars()->GetComponent(a,0);
            intensity[1] = processedPoints->GetPointData()->GetScalars()->GetComponent(a,1);
            intensity[2] = processedPoints->GetPointData()->GetScalars()->GetComponent(a,2);

            if((intensity[0]+intensity[1]+intensity[2])/3.0 < minPaintThreshold) //averge of each pixel if less than threshold incrament counter
            {
              missed++;
            }
          }//end iterate through points to check if painted

          if(missed/float(processedPoints->GetNumberOfPoints())>maxPassableNonpainted)//get ratio of missed spots, if below threshold set not painted
          {
            result_.painted[i]=false;
          }
          else
          {
            result_.painted[i]=true;
            if(added ==false)
            {
              result_.requiredPath.push_back(goal->path[curPathNum]);
              added = true;
            }
          }
          //******END TODO******//
          feedback_.percent =float(i)/float(num_meshes);
          simulation_service_.publishFeedback(feedback_);

        }//if not yet painted
      }//end each mesh
    }//end multiple paths

    if(success)
    {
      ROS_INFO("Succeeded");
      // set the action state to succeeded
      simulation_service_.setSucceeded(result_);
    }
  }//end executeCB
};//end class
}//end namespace



int main(int argc, char** argv)
{
  ros::init(argc, argv, "noether_simulator");

  noether_simulator::ProcessSimulatorNode node("noether_simulator");
  ros::spin();

  return 0;
}
