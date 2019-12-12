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
  void MakeMesh(const std::string& input, pcl::PointCloud<pcl::PointXYZ>& inMesh);
  void CleanMesh(const pcl::PointCloud<pcl::PointXYZ>& outMesh, pcl::PolygonMesh& outMeshPoly);
  bool SaveMesh(const pcl::PointCloud<pcl::PointXYZ>& outMesh, pcl::PolygonMesh& outMeshPoly, const std::string& outfile);
  bool GenerateCH(const std::string& infile, const std::string& outfile);
};
#endif
