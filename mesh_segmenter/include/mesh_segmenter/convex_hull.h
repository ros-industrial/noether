/**
 * @file convex_hull.h
 * @copyright Copyright (c) 2019, Southwest Research Institute
 *
 * @par License
 * Software License Agreement (Apache License)
 * @par
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * http://www.apache.org/licenses/LICENSE-2.0
 * @par
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

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
  /**
   * @brief default constructor
   */
  ConvexHullGenerator();
  /**
   * @brief Reads in a meash from a .PLY file
   * @param input The file to be read in
   * @param inMesh The mesh object that will be populated (internal)
   * @return Success of file read
   */
  bool MakeMesh(const std::string& input, pcl::PointCloud<pcl::PointXYZ>& inMesh);
  /**
   * @brief Reorients the inverted faces of the convex hull
   * @param outMesh Contains the vertices of the mesh
   * @param outMeshPoly Holds the polygon information for the convex hull
   */
  void CleanMesh(const pcl::PointCloud<pcl::PointXYZ>& outMesh, pcl::PolygonMesh& outMeshPoly);
  /**
   * @brief Saves convex hull to a PLY file
   * @param outMesh Contains the vertices of the mesh
   * @param outMeshPoly Holds the polygon information for the convex hull
   * @param outfile File of name to be written out
   * @return Success of file save
   */
  bool SaveMesh(const pcl::PointCloud<pcl::PointXYZ>& outMesh,
                pcl::PolygonMesh& outMeshPoly,
                const std::string& outfile);
  /**
   * @brief Master function that calls all other functions
   * @param infile Input file to be read in
   * @param outfile File of name to be written out
   * @return success Process completion
   */
  bool Generate(const std::string& infile, const std::string& outfile);
};
#endif
