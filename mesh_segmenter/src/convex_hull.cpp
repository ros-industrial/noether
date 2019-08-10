#include "ros/ros.h"
#include "noether_msgs/GenerateConvexHull.h"
#include <fstream>
#include <string>
#include<pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <pcl/surface/convex_hull.h>


using namespace std;



bool generate_ch(noether_msgs::GenerateConvexHull::Request& req,
	                 noether_msgs::GenerateConvexHull::Response& res)
{

  string inMeshFileName = req.file_in; //expects a path to file
  string modifier_ = "chull.ply";
  string outMeshFileName = inMeshFileName.substr(0, inMeshFileName.size()-4);
  outMeshFileName.append(modifier_);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr inMesh (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr outMesh (new pcl::PointCloud<pcl::PointXYZ>);
  
  cout<<"attempting to ingest file" << endl;
  //if its a .ply file
  pcl::PLYReader Reader;
  pcl::PLYWriter Writer;
  Reader.read(inMeshFileName, *inMesh); //populate inMesh
  	
  
  //boilerplate
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ConvexHull<pcl::PointXYZ> chull;
  
  chull.setInputCloud(inMesh); //generate hull
  chull.reconstruct(*outMesh); //save to outMesh

  cout<<inMeshFileName<<endl;
  cout<<outMeshFileName<<endl;

  Writer.write(outMeshFileName, *outMesh);

 return true;
}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "convex_hull_server");
  ros::NodeHandle nh;
  bool response;

  ros::ServiceServer service = nh.advertiseService("convex_hull_generation", generate_ch);

  ros::spin();

  return 0;
}