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

#include <vtk_viewer/vtk_utils.h>

class SegmentationAction
{
protected:
  ros::NodeHandle pnh_;
  actionlib::SimpleActionServer<noether_msgs::SegmentAction> server_;
  std::string action_name_;
  noether_msgs::SegmentFeedback feedback_;
  noether_msgs::SegmentResult result_;

public:
  SegmentationAction(ros::NodeHandle pnh, std::string name)
    : pnh_(pnh), server_(pnh_, name, boost::bind(&SegmentationAction::executeCB, this, _1), false), action_name_(name)
  {
    server_.start();
    ROS_INFO("Segmentation action server online");
  }

  ~SegmentationAction(void) {}

  void executeCB(const noether_msgs::SegmentGoalConstPtr& goal)
  {
    // Step 1: Load parameters
    double curvature_threshold = goal->segmentation_config.curvature_threshold;
    int min_cluster_size = goal->segmentation_config.min_cluster_size;
    int max_cluster_size = goal->segmentation_config.max_cluster_size;
    double neighborhood_radius = goal->segmentation_config.neighborhood_radius;
    bool use_mesh_normals = goal->segmentation_config.use_mesh_normals;

    auto& cfg = goal->filtering_config;
    int windowed_sinc_iterations = cfg.windowed_sinc_iterations;
    double windowed_sinc_pass_band = cfg.windowed_sinc_pass_band;
    double windowed_sinc_feature_angle = cfg.windowed_sinc_edge_angle;
    double windowed_sinc_edge_angle = cfg.windowed_sinc_edge_angle;
    bool windowed_sinc_edge_smoothing = cfg.windowed_sinc_edge_smoothing;
    bool windowed_sinc_boundary_smoothing = cfg.windowed_sinc_boundary_smoothing;
    bool windowed_sinc_nonmanifold_smoothing = cfg.windowed_sinc_nonmanifold_smoothing;
    bool windowed_sinc_normalize_coordinates = cfg.windowed_sinc_normalize_coordinates;

    int laplacian_iterations = cfg.laplacian_iterations;
    double laplacian_relaxation_factor = cfg.laplacian_relaxation_factor;
    double laplacian_edge_angle = cfg.laplacian_edge_angle;
    bool laplacian_edge_smoothing = cfg.laplacian_edge_smoothing;
    bool laplacian_boundary_smoothing = cfg.laplacian_boundary_smoothing;

    // Set defaults if invalid data given (or not set). Note: No guarantee of fitness of defaults
    curvature_threshold = (curvature_threshold < 0.0001) ? 0.05 : curvature_threshold;
    min_cluster_size = (min_cluster_size == 0) ? 500 : min_cluster_size;
    max_cluster_size = (max_cluster_size == 0) ? 1000000 : max_cluster_size;
    neighborhood_radius = (neighborhood_radius < 0.0001) ? 0.05 : neighborhood_radius;

    windowed_sinc_iterations = (windowed_sinc_iterations == 0) ? 20 : windowed_sinc_iterations;
    windowed_sinc_pass_band = (windowed_sinc_pass_band < 0.0001) ? 0.1 : windowed_sinc_pass_band;
    windowed_sinc_feature_angle = (windowed_sinc_feature_angle < 0.0001) ? 45. : windowed_sinc_feature_angle;
    windowed_sinc_edge_angle = (windowed_sinc_edge_angle < 0.0001) ? 15. : windowed_sinc_edge_angle;

    laplacian_iterations = (laplacian_iterations == 0) ? 10 : laplacian_iterations;
    laplacian_relaxation_factor = (laplacian_relaxation_factor < 0.0001) ? 0.1 : laplacian_relaxation_factor;
    laplacian_edge_angle = (laplacian_edge_angle < 0.0001) ? 15 : laplacian_edge_angle;

    // Convert ROS msg -> PCL Mesh -> VTK Mesh
    vtkSmartPointer<vtkPolyData> mesh;
    pcl::PolygonMesh input_pcl_mesh;
    pcl_conversions::toPCL(goal->input_mesh, input_pcl_mesh);
    vtk_viewer::pclEncodeMeshAndNormals(input_pcl_mesh, mesh, neighborhood_radius);
    if (use_mesh_normals)
    {
      ROS_INFO("Embedding Triangle Normals.");
      vtk_viewer::embedRightHandRuleNormals(mesh);
    }
    else
    {
      ROS_INFO("Calculating Cell Normals.");
      vtk_viewer::generateNormals(mesh);
    }

    // Step 2: Filter the mesh
    // Create some pointers - not used since passing with VTK pipeline but useful for debugging
    ROS_INFO("Beginning Filtering.");
    vtkSmartPointer<vtkPolyData> mesh_in = mesh;
    vtkSmartPointer<vtkPolyData> mesh_cleaned;
    vtkSmartPointer<vtkPolyData> mesh_filtered1;
    vtkSmartPointer<vtkPolyData> mesh_filtered2;

    vtkSmartPointer<vtkCleanPolyData> cleanPolyData = vtkSmartPointer<vtkCleanPolyData>::New();
    cleanPolyData->SetInputData(mesh_in);
    cleanPolyData->Update();
    mesh_cleaned = cleanPolyData->GetOutput();

    if (cfg.enable_filtering)
    {
      // Apply Windowed Sinc function interpolation Smoothing
      vtkSmartPointer<vtkWindowedSincPolyDataFilter> smooth_filter1 =
          vtkSmartPointer<vtkWindowedSincPolyDataFilter>::New();
      smooth_filter1->SetInputConnection(cleanPolyData->GetOutputPort());
      smooth_filter1->SetNumberOfIterations(windowed_sinc_iterations);
      smooth_filter1->SetPassBand(windowed_sinc_pass_band);
      windowed_sinc_edge_smoothing ? smooth_filter1->FeatureEdgeSmoothingOn() :
                                     smooth_filter1->FeatureEdgeSmoothingOff();  // Smooth along sharp interior edges
      smooth_filter1->SetFeatureAngle(windowed_sinc_feature_angle);  // Angle to identify sharp edges (degrees)
      smooth_filter1->SetEdgeAngle(windowed_sinc_edge_angle);        // Not sure what this controls (degrees)
      windowed_sinc_boundary_smoothing ? smooth_filter1->BoundarySmoothingOn() : smooth_filter1->BoundarySmoothingOff();
      windowed_sinc_nonmanifold_smoothing ? smooth_filter1->NonManifoldSmoothingOn() :
                                            smooth_filter1->NonManifoldSmoothingOff();
      windowed_sinc_normalize_coordinates ?
          smooth_filter1->NormalizeCoordinatesOn() :
          smooth_filter1->NormalizeCoordinatesOff();  // "Improves numerical stability"
      smooth_filter1->Update();
      mesh_filtered1 = smooth_filter1->GetOutput();

      // Apply Laplacian Smoothing
      // This moves the coordinates of each point toward the average of its adjoining points
      vtkSmartPointer<vtkSmoothPolyDataFilter> smooth_filter2 = vtkSmartPointer<vtkSmoothPolyDataFilter>::New();
      smooth_filter2->SetInputConnection(smooth_filter1->GetOutputPort());
      smooth_filter2->SetNumberOfIterations(laplacian_iterations);
      smooth_filter2->SetRelaxationFactor(laplacian_relaxation_factor);
      smooth_filter2->SetEdgeAngle(laplacian_edge_angle);
      laplacian_edge_smoothing ? smooth_filter2->FeatureEdgeSmoothingOn() : smooth_filter2->FeatureEdgeSmoothingOff();
      laplacian_boundary_smoothing ? smooth_filter2->BoundarySmoothingOn() : smooth_filter2->BoundarySmoothingOff();
      smooth_filter2->Update();
      mesh_filtered2 = smooth_filter2->GetOutput();
    }

    // Step 3: Segment the mesh
    mesh_segmenter::MeshSegmenter segmenter;
    if (cfg.enable_filtering)
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
    // TODO: Since this can take a long time to run, it would be nice if the action was able to give feedback in the
    // form of completion percentage while this is running. I believe this could be roughly done by monitoring i in
    // MeshSegmenter::segmentMesh() in a seperate thread and updating the feedback there

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
  ros::NodeHandle pnh("~");
  vtkObject::GlobalWarningDisplayOff();
  pcl::console::setVerbosityLevel(pcl::console::L_ALWAYS);
  SegmentationAction segmenter(pnh, "segmenter");
  ros::spin();

  return 0;
}
