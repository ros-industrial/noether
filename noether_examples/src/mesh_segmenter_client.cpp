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

#include <noether_msgs/SegmentAction.h>
#include <actionlib/client/service_client.h>
#include <pcl_conversions/pcl_conversions.h>

#include <noether_examples/noether_examples.h>
#include <vtkSTLWriter.h>
#include <ros/package.h>

namespace noether
{
void Noether::addMeshDisplay(std::vector<vtkSmartPointer<vtkPolyData> >& meshes)
{
  // mesh colors should be darker than path colors
  int colors[] = {
    0xcc0000, 0xcc6500, 0xcccc00, 0x65cc00, 0x00cc00, 0x00cc65,
    0x00cccc, 0x0065cc, 0x0000cc, 0x6500cc, 0xcc00cc, 0xcc0065,
  };

  size_t size;
  size = sizeof(colors) / sizeof(colors[0]);

  for (int i = 0; i < meshes.size(); ++i)
  {
    std::vector<float> color(3);
    color[2] = float(colors[i % size] & 0xff) / 255.0;
    color[1] = float((colors[i % size] & 0xff00) >> 8) / 255.0;
    color[0] = float((colors[i % size] & 0xff0000) >> 16) / 255.0;

    viewer_.addPolyDataDisplay(meshes[i], color);
  }
}
}  // namespace noether

// Called once when the goal completes
void doneCb(const actionlib::SimpleClientGoalState& state, const noether_msgs::SegmentResultConstPtr& result)
{
  ROS_INFO("Finished in state [%s]", state.toString().c_str());

  ros::shutdown();
}

void activeCb() { ROS_INFO("Goal just went active"); }

// Called every time feedback is received for the goal
void feedbackCb(const noether_msgs::SegmentFeedbackConstPtr& feedback) { ROS_INFO("Got Feedback"); }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "noether_node");
  ros::NodeHandle pnh("~");

  // Step 1: Load parameters
  std::string file;
  double curvature_threshold, neighborhood_size;
  int min_cluster_size, max_cluster_size;
  bool show_individually, save_outputs, use_mesh_normals;
  pnh.param<std::string>("filename", file, "");
  pnh.param<int>("min_cluster_size", min_cluster_size, 500);
  pnh.param<int>("max_cluster_size", max_cluster_size, 1000000);
  pnh.param<double>("curvature_threshold", curvature_threshold, 0.3);
  pnh.param<double>("neighborhood_size", neighborhood_size, 0.05);
  pnh.param<bool>("use_mesh_normals", use_mesh_normals, true);
  pnh.param<bool>("show_individually", show_individually, false);
  pnh.param<bool>("save_outputs", save_outputs, false);

  // Step 2: Import the mesh
  pcl::PolygonMesh pcl_mesh;
  pcl::io::loadPLYFile(file, pcl_mesh);
  ROS_INFO_STREAM("Imported as PCL mesh of size " << pcl_mesh.cloud.data.size() << '\n');

  // Step 3: Use action interface
  actionlib::SimpleActionClient<noether_msgs::SegmentAction> action("mesh_segmenter_server_node/segmenter", true);
  ROS_INFO("Waiting for action server to start.");
  action.waitForServer();  // will wait for infinite time

  ROS_INFO("Action server started, sending goal.");

  // Convert to ROS message
  pcl_msgs::PolygonMesh pcl_mesh_msgs;
  pcl_conversions::fromPCL(pcl_mesh, pcl_mesh_msgs);

  // Set Goal
  noether_msgs::SegmentGoal msg;
  msg.input_mesh = pcl_mesh_msgs;

  msg.segmentation_config.max_cluster_size = max_cluster_size;
  msg.segmentation_config.min_cluster_size = min_cluster_size;
  msg.segmentation_config.curvature_threshold = curvature_threshold;
  msg.segmentation_config.neighborhood_radius = neighborhood_size;
  msg.segmentation_config.use_mesh_normals = use_mesh_normals;
  msg.filtering_config.enable_filtering = true;
  msg.filtering_config.windowed_sinc_iterations = 20;
  ros::Time tStart = ros::Time::now();
  // Note this will take a long time. It will also be blocking unless you have started an async spinner
  action.sendGoal(msg, &doneCb, &activeCb, &feedbackCb);

  // wait for the action to return
  bool finished_before_timeout = action.waitForResult(ros::Duration(600.0));

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

  // Step 4: Convert ROS msg -> PCL -> VTK
  ROS_INFO("Converting ROS msg to VTK");
  for (int ind = 0; ind < result_msgs.size(); ind++)
  {
    pcl::PolygonMesh tmp;
    pcl_conversions::toPCL(result_msgs[ind], tmp);

    vtkSmartPointer<vtkPolyData> segmented_mesh;
    pcl::VTKUtils::convertToVTK(tmp, segmented_mesh);
    segmented_meshes.push_back(segmented_mesh);
  }

  // Step 5: Display mesh
  ROS_INFO_STREAM("Displaying " << segmented_meshes.size() << " meshes \n");
  ROS_INFO("Close VTK window to continue");
  if (show_individually)
  {
    for (int ind = 0; ind < segmented_meshes.size(); ind++)
    {
      if (true)
      {
        ROS_INFO_STREAM("Mesh: " << ind << "\n");
        std::vector<vtkSmartPointer<vtkPolyData> > tmp(1);
        tmp.push_back(segmented_meshes[ind]);
        noether::Noether viz;
        viz.addMeshDisplay(tmp);
        viz.visualizeDisplay();
      }
    }
  }
  else
  {
    // Extract panels and edges
    std::vector<vtkSmartPointer<vtkPolyData> > panels(segmented_meshes.begin(), segmented_meshes.end() - 1);
    std::vector<vtkSmartPointer<vtkPolyData> > edges(1);
    edges.push_back(segmented_meshes.back());

    noether::Noether viz;
    ROS_INFO("Displaying Segments");
    viz.addMeshDisplay(segmented_meshes);
    viz.visualizeDisplay();
  }

  // Step 6: Save output meshes
  if (save_outputs)
  {
    for (int ind = 0; ind < segmented_meshes.size(); ind++)
    {
      ROS_INFO_STREAM("Saving: " << ind << "\n");

      std::ostringstream ss;
      ss << ind;
      std::string filename = ros::package::getPath("noether_examples") + "/meshes/outputs/output_" + ss.str() + ".stl";

      vtkSmartPointer<vtkSTLWriter> stlWriter = vtkSmartPointer<vtkSTLWriter>::New();
      stlWriter->SetFileName(filename.c_str());
      stlWriter->SetInputData(segmented_meshes[ind]);
      stlWriter->Write();
    }
  }

  return 0;
}
