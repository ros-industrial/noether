#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <noether_msgs/SegmentAction.h>
#include <mesh_segmenter/mesh_segmenter.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>


#include <vtkPolyDataNormals.h>
#include <vtkCleanPolyData.h>
#include <vtkWindowedSincPolyDataFilter.h>
#include <vtkSmoothPolyDataFilter.h>

#include <noether/noether.h>
#include <vtk_viewer/vtk_utils.h>

class SegmentationAction
{
protected:
  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<noether_msgs::SegmentAction> server_;
  std::string action_name_;
  noether_msgs::SegmentFeedback feedback_;
  noether_msgs::SegmentResult result_;

public:
  SegmentationAction(ros::NodeHandle nh, std::string name)
    : server_(nh_, name, boost::bind(&SegmentationAction::executeCB, this, _1), false), action_name_(name), nh_(nh)
  {
    server_.start();
  }

  ~SegmentationAction(void) {}

  void executeCB(const noether_msgs::SegmentGoalConstPtr& goal)
  {
    // Step 1: Load parameters
    double curvature_threshold;
    int min_cluster_size, max_cluster_size;
    nh_.param<int>("min_cluster_size", min_cluster_size, 500);
    nh_.param<int>("max_cluster_size", max_cluster_size, 1000000);
    nh_.param<double>("curvature_threshold", curvature_threshold, 0.3);

    // Convert ROS msg -> PCL Mesh -> VTK Mesh
    vtkSmartPointer<vtkPolyData> mesh;
    pcl::PolygonMesh input_pcl_mesh;
    pcl_conversions::toPCL(goal->input_mesh, input_pcl_mesh);
    vtk_viewer::pclEncodeMeshAndNormals(input_pcl_mesh, mesh);
//    pcl::VTKUtils::convertToVTK(input_pcl_mesh, mesh);            // Converts w



    // Step 2: Filter the mesh
    // Create some pointers - not used since passing with "ports" but useful for debugging
    ROS_INFO("Beginning Filtering.");
    vtkSmartPointer<vtkPolyData> mesh_in = mesh;
    vtkSmartPointer<vtkPolyData> mesh_cleaned;
    vtkSmartPointer<vtkPolyData> mesh_filtered1;
    vtkSmartPointer<vtkPolyData> mesh_filtered2;

    // Remove duplicate points
    vtkSmartPointer<vtkCleanPolyData> cleanPolyData = vtkSmartPointer<vtkCleanPolyData>::New();
    cleanPolyData->SetInputData(mesh_in);
    cleanPolyData->Update();
    mesh_cleaned = cleanPolyData->GetOutput();

    if (goal->filter)
    {
      // Apply Windowed Sinc function interpolation Smoothing
      vtkSmartPointer<vtkWindowedSincPolyDataFilter> smooth_filter1 =
          vtkSmartPointer<vtkWindowedSincPolyDataFilter>::New();
      smooth_filter1->SetInputConnection(cleanPolyData->GetOutputPort());
//      smooth_filter1->SetInputData(mesh_cleaned);
      smooth_filter1->SetNumberOfIterations(20);
      smooth_filter1->SetPassBand(0.1);
      smooth_filter1->FeatureEdgeSmoothingOff();  // Smooth along sharp interior edges
      smooth_filter1->SetFeatureAngle(45);        // Angle to identify sharp edges (degrees)
      smooth_filter1->SetEdgeAngle(15);           // Not sure what this controls (degrees)
      smooth_filter1->BoundarySmoothingOff();
      smooth_filter1->NonManifoldSmoothingOff();
      smooth_filter1->NormalizeCoordinatesOn();  // "Improves numerical stability"
      smooth_filter1->Update();
      mesh_filtered1 = smooth_filter1->GetOutput();

      // Apply Laplacian Smoothing
      // This moves the coordinates of each point toward the average of its adjoining points
      vtkSmartPointer<vtkSmoothPolyDataFilter> smooth_filter2 = vtkSmartPointer<vtkSmoothPolyDataFilter>::New();
      smooth_filter2->SetInputConnection(smooth_filter1->GetOutputPort());
      //  smooth_filter2->SetInputData(mesh_cleaned);
      smooth_filter2->SetNumberOfIterations(10);
      smooth_filter2->SetRelaxationFactor(0.1);
      //  smooth_filter2->SetEdgeAngle(somenumber);
      smooth_filter2->FeatureEdgeSmoothingOff();
      smooth_filter2->BoundarySmoothingOff();
      smooth_filter2->Update();
      mesh_filtered2 = smooth_filter2->GetOutput();
    }

    // Step 3: Segment the mesh
    mesh_segmenter::MeshSegmenter segmenter;
    ROS_INFO("Displaying mesh");
    noether::Noether viz;
    std::vector <vtkSmartPointer<vtkPolyData> > tmp;
    tmp.push_back((mesh_filtered2));
    viz.addMeshDisplay(tmp);
    viz.visualizeDisplay();

    if (goal->filter)
      segmenter.setInputMesh(mesh_filtered2);
    else
      segmenter.setInputMesh(mesh_cleaned);
    segmenter.setMinClusterSize(min_cluster_size);
    segmenter.setMaxClusterSize(max_cluster_size);
    segmenter.setCurvatureThreshold(curvature_threshold);

    ROS_INFO("Beginning Segmentation.");
    ros::Time tStart = ros::Time::now();
    segmenter.segmentMesh();
    ROS_INFO("Segmentation time: %.3f", (ros::Time::now() - tStart).toSec());

    // Get Mesh segment
    std::vector<vtkSmartPointer<vtkPolyData> > segmented_meshes = segmenter.getMeshSegments();
    //    std::vector<vtkSmartPointer<vtkPolyData> > panels(segmented_meshes.begin(), segmented_meshes.end() - 1);
    //    std::vector<vtkSmartPointer<vtkPolyData> > edges(1);
    //    edges.push_back(segmented_meshes.back());

    // Step 4: Convert to PolygonMesh
    std::vector<pcl_msgs::PolygonMesh> pcl_mesh_msgs;
    pcl_mesh_msgs.reserve(segmented_meshes.size());
    for (int ind = 0; ind < segmented_meshes.size(); ind++)
    {
      pcl::PolygonMesh pcl_mesh;
      pcl::VTKUtils::convertToPCL(segmented_meshes[ind], pcl_mesh);
      pcl_conversions::fromPCL(pcl_mesh, pcl_mesh_msgs[ind]);
    }

    // Step 5: Return result
    bool success = true;
    if (server_.isPreemptRequested() || !ros::ok())
    {
      ROS_INFO("%s: Preempted", action_name_.c_str());
      // set the action state to preempted
      server_.setPreempted();
      success = false;
    }
    if (success)
    {
      result_.output_mesh = pcl_mesh_msgs;
      ROS_INFO("%s: Succeeded", action_name_.c_str());
      // set the action state to succeeded
      server_.setSucceeded(result_);
    }
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "segmentation_server");
  ros::NodeHandle nh;

  SegmentationAction segmenter(nh, "segmenter");
  ros::spin();

  return 0;
}
