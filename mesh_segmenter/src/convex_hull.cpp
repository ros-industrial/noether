#include <ros/ros.h>
#include "noether_msgs/GenerateConvexHull.h"
#include <fstream>
#include <string>
#include <pcl/PolygonMesh.h>
#include <pcl/point_types.h>
#include <pcl/conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/vtk_lib_io.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/common/centroid.h>
#include <Eigen/Dense>


using namespace std;



bool generate_ch(noether_msgs::GenerateConvexHull::Request& req,
	                 noether_msgs::GenerateConvexHull::Response& res)
{

  string inMeshFileName = req.file_in; //expects a path to file
  string modifier_ = "_chullt.ply";
  string outMeshFileName = inMeshFileName.substr(0, inMeshFileName.size()-4);
  outMeshFileName.append(modifier_);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr inMesh (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZ>::Ptr outMesh (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PolygonMesh::Ptr outMeshPoly (new pcl::PolygonMesh);

  
  //if its a .ply file
  pcl::PLYReader Reader;
  Reader.read(inMeshFileName, *inMesh); //populate inMesh
  	
  
  //boilerplate
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_hull (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::ConvexHull<pcl::PointXYZ> chull;
  
  chull.setInputCloud(inMesh); //generate hull
  chull.reconstruct(*outMesh, outMeshPoly->polygons); //save to outMesh
  

  //find centroid coords by finding average x, y, z

  Eigen::Matrix< float, 4, 1 > mid; //kinda need this to be a vector
  int centroid_success = pcl::compute3DCentroid(*outMesh, mid);
  Eigen::Vector3d midVec= {mid[0], mid[1], mid[2]};

  //invert bad polygons
  for (int t=0; t < (outMeshPoly->polygons.size()); t++)
  {
    pcl::Vertices verts;
    verts = outMeshPoly->polygons[t];

    pcl::PointXYZ a = outMesh->points[verts.vertices[0]];
    pcl::PointXYZ b = outMesh->points[verts.vertices[1]];
    pcl::PointXYZ c = outMesh->points[verts.vertices[2]];


    Eigen::Vector3d p0 = {a.x, a.y, a.z};
    Eigen::Vector3d p1 = {b.x, b.y, b.z};
    Eigen::Vector3d p2 = {c.x, c.y, c.z};

    Eigen::Vector3d v1 = p1 - p0; 
    Eigen::Vector3d v2 = p2 - p0;
    Eigen::Vector3d d = p0 - midVec;
    Eigen::Vector3d normal = v1.cross(v2);
    float works = d.dot(normal);

    if (works < 0)
    {
      int temp;
      temp = verts.vertices[1];
      verts.vertices[1] = verts.vertices[2];
      verts.vertices[2] = temp;

    }
    outMeshPoly->polygons[t] = verts;
  } 
  pcl::toPCLPointCloud2(*outMesh, outMeshPoly->cloud);
  pcl::io::savePolygonFile(outMeshFileName, *outMeshPoly, false);

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
