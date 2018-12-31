#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <noether_msgs/SegmentAction.h>
#include <mesh_segmenter/mesh_segmenter.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/vtk_lib_io.h>

#include <vtkPolyDataNormals.h>
#include <vtkCleanPolyData.h>
#include <vtkWindowedSincPolyDataFilter.h>
#include <vtkSmoothPolyDataFilter.h>

//#include <noether/noether.h>
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
    ROS_INFO("Segmentation action server online");
  }

  ~SegmentationAction(void) {}

  void executeCB(const noether_msgs::SegmentGoalConstPtr& goal)
  {
    // Step 1: Load parameters
    // TODO: Pass these in along with action
    std::string file;
    double curvature_threshold;
    int min_cluster_size, max_cluster_size;
    bool show_individually, save_outputs;
    nh_.param<std::string>("/mesh_segmenter_client_node/filename", file, "");
    nh_.param<int>("/mesh_segmenter_client_node/min_cluster_size", min_cluster_size, 500);
    nh_.param<int>("/mesh_segmenter_client_node/max_cluster_size", max_cluster_size, 1000000);
    nh_.param<double>("/mesh_segmenter_client_node/curvature_threshold", curvature_threshold, 0.3);
    nh_.param<bool>("/mesh_segmenter_client_node/show_individually", show_individually, false);
    nh_.param<bool>("/mesh_segmenter_client_node/save_outputs", save_outputs, false);

    // Convert ROS msg -> PCL Mesh -> VTK Mesh
    vtkSmartPointer<vtkPolyData> mesh;
    pcl::PolygonMesh input_pcl_mesh;
    pcl_conversions::toPCL(goal->input_mesh, input_pcl_mesh);
    vtk_viewer::pclEncodeMeshAndNormals(input_pcl_mesh, mesh);  //, 100, pcl::PointXYZ(0, 50.0, 50.0));
    vtk_viewer::generateNormals(mesh);

    // Step 2: Filter the mesh
    // Create some pointers - not used since passing with VTK pipeline but useful for debugging
    ROS_INFO("Beginning Filtering.");
    vtkSmartPointer<vtkPolyData> mesh_in = mesh;
    vtkSmartPointer<vtkPolyData> mesh_cleaned;
    vtkSmartPointer<vtkPolyData> mesh_filtered1;
    vtkSmartPointer<vtkPolyData> mesh_filtered2;

    /*
     * This is left in to show how to do this in case we need to later. Cleaning the mesh seems like a logical thing to
     * do, and I only disabled it because it seemed to cause some issues. I'm hopeful after I switch to MLS for
     * curvature calculations it will solve some of these problems, and this can be on by default.
     *
     * When this is done line 87 can be removed and line 86 can be uncommented.
     * 12/31/2018
    // Remove duplicate points (Note: this seems to cause problems sometimes)
    //    vtkSmartPointer<vtkCleanPolyData> cleanPolyData = vtkSmartPointer<vtkCleanPolyData>::New();
    //    cleanPolyData->SetInputData(mesh_in);
    //    cleanPolyData->Update();
    //    mesh_cleaned = cleanPolyData->GetOutput();
*/
    vtk_viewer::generateNormals(mesh_in);

    if (goal->filter)
    {
      // Apply Windowed Sinc function interpolation Smoothing
      vtkSmartPointer<vtkWindowedSincPolyDataFilter> smooth_filter1 =
          vtkSmartPointer<vtkWindowedSincPolyDataFilter>::New();
      //      smooth_filter1->SetInputConnection(cleanPolyData->GetOutputPort());       // See note above
      smooth_filter1->SetInputData(mesh_in);
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
      smooth_filter2->SetNumberOfIterations(10);
      smooth_filter2->SetRelaxationFactor(0.1);
      //  smooth_filter2->SetEdgeAngle(somenumber);     // This is left as an example of a useful filter that could be applied
      smooth_filter2->FeatureEdgeSmoothingOff();
      smooth_filter2->BoundarySmoothingOff();
      smooth_filter2->Update();
      mesh_filtered2 = smooth_filter2->GetOutput();
    }

    // Step 3: Segment the mesh
    mesh_segmenter::MeshSegmenter segmenter;
    if (goal->filter)
      segmenter.setInputMesh(mesh_filtered2);
    else
      segmenter.setInputMesh(mesh_in);
    segmenter.setMinClusterSize(min_cluster_size);
    segmenter.setMaxClusterSize(max_cluster_size);
    segmenter.setCurvatureThreshold(curvature_threshold);

    ROS_INFO("Beginning Segmentation.");
    ros::Time tStart = ros::Time::now();
    segmenter.segmentMesh();
    ROS_INFO("Segmentation time: %.3f", (ros::Time::now() - tStart).toSec());

    // Get Mesh segment
    std::vector<vtkSmartPointer<vtkPolyData> > segmented_meshes = segmenter.getMeshSegments();

    // Step 4: Convert to PolygonMesh
    std::vector<pcl_msgs::PolygonMesh> pcl_mesh_msgs;
    for (int ind = 0; ind < segmented_meshes.size(); ind++)
    {
      pcl::PolygonMesh pcl_mesh;
      pcl::io::vtk2mesh(segmented_meshes[ind], pcl_mesh);
      pcl_msgs::PolygonMesh pcl_mesh_msg;
      pcl_conversions::fromPCL(pcl_mesh, pcl_mesh_msg);
      pcl_mesh_msgs.push_back(pcl_mesh_msg);
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
