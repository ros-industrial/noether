/*
 * Copyright (c) 2016, Southwest Research Institute
 * All rights reserved.
 *
 */

#include <ros/ros.h>

#include <gtest/gtest.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/point_traits.h>
#include <pcl/point_types.h>
#include <pcl/common/io.h>
#include <pcl/console/print.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/ascii_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <fstream>
#include <locale>
#include <stdexcept>

#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <vtkSmartPointer.h>
#include <vtkPolyData.h>

//#include "noether/noether.h"
//#include <mesh_segmenter/mesh_segmenter.h>
//#include <vtkPointData.h>
//#include <ros/ros.h>
//#include <ros/file_log.h>

////#include <vtkPolyDataNormals.h>
////#include <vtkCleanPolyData.h>
////#include <vtkWindowedSincPolyDataFilter.h>
////#include <vtkSmoothPolyDataFilter.h>

#include <noether_msgs/SegmentAction.h>
#include <actionlib/client/service_client.h>
#include <pcl_conversions/pcl_conversions.h>
//#include <pcl/surface/vtk_smoothing/vtk_utils.h>
//#include <pcl/io/ply_io.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "noether_node");
  ros::NodeHandle pnh("~");

  ROS_ERROR("Press enter to continue");
  std::cin.ignore(std::numeric_limits<std::streamsize>::max(), '\n');

  // Step 1: Load the 'filename' parameter
  std::string filename;
  pnh.param<std::string>("filename", filename, "");
  std::cout << filename << '\n';

  // Step 2: Import the mesh
  pcl::PolygonMesh pcl_mesh;
  pcl::io::loadPLYFile(filename, pcl_mesh);
  std::cout << "Imported as PCL mesh of size " << pcl_mesh.cloud.data.size() << '\n' ;

  // Step 3: Use action interface
  actionlib::SimpleActionClient<noether_msgs::SegmentAction> action("segmenter", true);
  ROS_INFO("Waiting for action server to start.");
  action.waitForServer();  // will wait for infinite time

  ROS_INFO("Action server started, sending goal.");
  // Convert to ROS message
  pcl_msgs::PolygonMesh pcl_mesh_msgs;
  pcl_conversions::fromPCL(pcl_mesh, pcl_mesh_msgs);
  // Set Goal
  noether_msgs::SegmentGoal msg;
  msg.input_mesh = pcl_mesh_msgs;
  msg.filter = true;
  ros::Time tStart = ros::Time::now();
  action.sendGoal(msg);

  // wait for the action to return
  bool finished_before_timeout = action.waitForResult(ros::Duration(300.0));

  std::vector<pcl_msgs::PolygonMesh> result_msgs;
  std::vector<vtkSmartPointer<vtkPolyData> > segmented_meshes;
  if (finished_before_timeout)
  {
    actionlib::SimpleClientGoalState state = action.getState();
    ROS_INFO("Action finished: %s", state.toString().c_str());
    ROS_INFO("Segmentation time: %.3f", (ros::Time::now() - tStart).toSec());
    noether_msgs::SegmentResultConstPtr result = action.getResult();
    result_msgs = result->output_mesh;
  }
  else
    ROS_INFO("Action did not finish before the time out.");
  for (int ind = 0; ind < result_msgs.size(); ind++)
  {
    pcl::PolygonMesh tmp;
    pcl_conversions::toPCL(result_msgs[ind], tmp);
    pcl::VTKUtils::convertToVTK(tmp, segmented_meshes[ind]);
  }

  // Step 4: Convert ROS msg -> PCL -> VTK



//  // Get Mesh segment
//  //  std::vector<vtkSmartPointer<vtkPolyData> > segmented_meshes = segmenter.getMeshSegments();
//    std::vector<vtkSmartPointer<vtkPolyData> > panels(segmented_meshes.begin(), segmented_meshes.end() - 1);
//    std::vector<vtkSmartPointer<vtkPolyData> > edges(1);
//    edges.push_back(segmented_meshes.back());

//    ROS_INFO("Displaying Edges");
//    noether::Noether viz;
//    viz.addMeshDisplay(edges);
//    viz.visualizeDisplay();
//    ROS_INFO("Displaying Segments");
//    viz.addMeshDisplay(panels);
//    viz.visualizeDisplay();
//    ROS_INFO("Displaying Both");
//    viz.addMeshDisplay(segmented_meshes);
//    viz.visualizeDisplay();

  return 0;
}
