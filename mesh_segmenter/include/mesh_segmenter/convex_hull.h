#ifndef CONVEX_HULL_H
#define CONVEX_HULL_H

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

class ConvexHullGenerator
{
public:
  ConvexHullGenerator(); //constructor
  void makemesh(string input, pcl::PointCloud<pcl::PointXYZ> *inMesh);	
  bool cleanmesh(pcl::PointCloud<pcl::PointXYZ> outMesh. pcl::PolygonMesh::Ptr outMeshPoly);	
  bool savemesh(pcl::PointCloud<pcl::PointXYZ>::Ptr outMesh, pcl::PolygonMesh::Ptr outMeshPoly, string outfile);
  bool generate_ch(string Infile, String Outfile);	

private: //What needs to be protected?
	pcl::PointCloud<pcl::PointXYZ>::Ptr inMesh;
    pcl::PointCloud<pcl::PointXYZ>::Ptr outMesh;
    pcl::PolygonMesh::Ptr outMeshPoly;
    pcl::ConvexHull<pcl::PointXYZ> chull;
}
#endif