#include <ros/ros.h>
#include <mesh_segmenter/convex_hull.h>
#include <noether_msgs/GenerateConvexHull.h>	
  
bool executeCB(const noether_msgs::GenerateConvexHull::Request& req)
  {
  	ConvexHullGenerator ch_gen;

     //step 1: Read in file
  	 string inMeshFileName_ = req.file_in; //expects a path to file
     string modifier_ = "_chullt.ply";
     outMeshFileName_ = inMeshFileName_.substr(0, inMeshFileName_.size()-4);
     outMeshFileName_.append(modifier_);
     
     //step 2: Call library
  	 bool success = chgen.generate_ch(inMeshFileName_,outMeshFileName_);

  	 return(success);
  }

int main(int argc, char **argv)
{
  ros::init(argc, argv, "convex_hull_server");
  ros::NodeHandle nh;

  HullService server(nh);
  ros::ServiceServer service = nh.advertiseService("convex_hull_generation", executeCB);
  ros::spin();
}