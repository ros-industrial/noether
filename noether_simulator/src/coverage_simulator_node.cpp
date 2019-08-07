/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2016, Southwest Research Institute
 *
 * file thickness_simulator_node.cpp
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
#include "noether_msgs/SimulateCoverageAction.h"

#include <tool_path_planner/tool_path_planner_base.h>
#include <tool_path_planner/raster_tool_path_planner.h>

namespace noether_simulator {
class ProcessSimulatorNode{
private:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<noether_msgs::SimulateCoverageAction> simulation_service_;

  double tool_height_;
  double process_rate_;
  double display_sigma_;
  double tool_radius_;

  void updateParameters(noether_simulator::NoetherSimulatorConfig& config, uint32_t level);
  dynamic_reconfigure::Server<noether_simulator::NoetherSimulatorConfig> reconfigurer_;
  boost::mutex param_lock_;
  double vect_[3], center_[3];
  bool debug_on_;
  std::string log_directory_;
  noether_msgs::SimulateCoverageFeedback feedback_;
  noether_msgs::SimulateCoverageResult result_;

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
  void executeCB(const noether_msgs::SimulateCoverageGoalConstPtr &goal)
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
    nh_.param("/noether_simulator/pt_spacing",tool.pt_spacing, 0.5);
    nh_.param("/noether_simulator/line_spacing",tool.line_spacing,10.75);
    nh_.param("/noether_simulator/tool_offset",tool.tool_offset, 0.0);
    nh_.param("/noether_simulator/intersecting_plane_hiehgt",tool.intersecting_plane_height,0.15);
    nh_.param("/noether_simulator/nearest_neighbors",tool.simulator_nearest_neighbors,30);
    nh_.param("/noether_simulator/min_hole_size",tool.min_hole_size,0.1);
    nh_.param("/noether_simulator/min_segment_size",tool.min_segment_size,1.0);
    nh_.param("/noether_simulator/raster_angle",tool.raster_angle,0.0);
    nh_.param("/noether_simulator/raster_wrt_global_axes",tool.raster_wrt_global_axes ,false);
    nh_.param("/noether_simulator/tool_radius",tool.simulator_tool_radius,1.0);
    nh_.param("/noether_simulator/tool_height",tool.simulator_tool_height,2.0);
    sim.setTool(tool);

    for(int i=0; i<num_meshes;i++)
    {
      result_.coverage.push_back(false);
    }
    //start multiple paths
    for (int curPathNum=0;curPathNum<goal->path.size();curPathNum++)
    {
      if(result_.coverage[curPathNum]==false)
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

        for(int i=0; i <num_meshes; i++)//run simulator for each mesh
        {
          if(simulation_service_.isPreemptRequested() || !ros::ok())
          {
            simulation_service_.setPreempted();
            success = false;
            break;
          }
          if(result_.coverage[i]==false)//if not yet covered
          {

            //convert mesh for simulator
            pcl::PolygonMesh surface_mesh;
            pcl_conversions::toPCL(goal->input_mesh[i], surface_mesh);
            vtkSmartPointer<vtkPolyData> surface_mesh_vtk = vtkSmartPointer<vtkPolyData>::New();
            pcl::VTKUtils::convertToVTK(surface_mesh, surface_mesh_vtk);

            sim.setInputMesh(surface_mesh_vtk);//add mesh to simulator
            sim.runSimulation();//display covered parts

            //deterimine if covered enough
            vtkSmartPointer<vtkPolyData> processedPoints = vtkSmartPointer<vtkPolyData>::New();
            processedPoints = sim.getSimulatedPoints();//get a copy of simulated points to operate on
            vtkIdType length = processedPoints->GetNumberOfPoints();

            double intensity[3] = {0,0,0};//setup flann
            int missed = 0;
            double minPaintThreshold = 5;
            double maxPassableNoncoverage = 0.1;
            double p [3];
            vtkSmartPointer<vtkKdTreePointLocator> pointTree =
                vtkSmartPointer<vtkKdTreePointLocator>::New();
            pointTree->SetDataSet(processedPoints);
            pointTree->BuildLocator();
            unsigned int k = 30;
            double currPoint [3];

            for(vtkIdType a = 0; a < length; a++)//iterate through points to check if covered
            {
              intensity[0] =0;//reseting intensity for next point in mesh
              intensity[1] =0;
              intensity[2] =0;

              vtkSmartPointer<vtkIdList> result = vtkSmartPointer<vtkIdList>::New();
              processedPoints->GetPoint(a,currPoint);
              pointTree->FindClosestNPoints(k, currPoint, result);//flann

              for (vtkIdType i = 0; i < k; i++)//iterate through k closest points average intensities
              {
                vtkIdType point_ind = result->GetId(i);

                //find average color intensity for each neighorhood
                intensity[0] += processedPoints->GetPointData()->GetScalars()->GetComponent(point_ind,0);
                intensity[1] += processedPoints->GetPointData()->GetScalars()->GetComponent(point_ind,1);
                intensity[2] += processedPoints->GetPointData()->GetScalars()->GetComponent(point_ind,2);
              }

              if((intensity[0])/float(k)+(intensity[1])/float(k)+(intensity[2])/30 < minPaintThreshold) //averge of each pixel if less than threshold incrament counter
              {
                missed++;
              }
            }//end iterate through points to check if covered

            if(missed/float(processedPoints->GetNumberOfPoints())>maxPassableNoncoverage)//get ratio of missed spots, if below threshold set not covered
            {
              result_.coverage[i]=false;
            }
            else
            {
              result_.coverage[i]=true;
              if(added ==false)
              {
                result_.requiredPath.push_back(goal->path[curPathNum]);
                added = true;
              }
            }
            feedback_.percent =float(i)/float(num_meshes);
            simulation_service_.publishFeedback(feedback_);

          }//if not yet covered
        }//end each mesh
      }//end if path used
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
