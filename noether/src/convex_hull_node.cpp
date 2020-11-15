#include <ros/ros.h>
#include <mesh_segmenter/convex_hull.h>
#include <noether_msgs/GenerateConvexHull.h>

bool executeCB(noether_msgs::GenerateConvexHull::Request& req, noether_msgs::GenerateConvexHull::Response& res)
{
  ConvexHullGenerator ch_gen;

  // step 1: Read in file
  std::string inMeshFileName = req.file_in;  // expects a path to file
  std::string modifier = "_chull.ply";
  std::string outMeshFileName = inMeshFileName.substr(0, inMeshFileName.size() - 4);
  outMeshFileName.append(modifier);

  // step 2: Call library
  bool success = ch_gen.Generate(inMeshFileName, outMeshFileName);

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "convex_hull_server");
  ros::NodeHandle nh;

  ros::ServiceServer service = nh.advertiseService("convex_hull_generation", executeCB);
  ros::spin();
}
