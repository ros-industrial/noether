/*
 * Copyright (c) 2016, Southwest Research Institute
 * All rights reserved.
 *
 */

#ifndef PATH_PLANNER_H
#define PATH_PLANNER_H

#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkParametricSpline.h>

namespace path_planner
{
  struct ProcessPath
  {
    vtkSmartPointer<vtkPolyData> line;
    vtkSmartPointer<vtkParametricSpline> spline;
    vtkSmartPointer<vtkPolyData> derivatives;
    vtkSmartPointer<vtkPolyData> intersection_plane;
  };

  struct ProcessTool
  {
    double pt_spacing;
    double line_spacing;
    double tool_offset;
    double intersecting_plane_height;
    int nearest_neighbors; // not sure if this should be a part of the tool or derived from
                           // the pt spacing, line spacing, and mesh density
  };

  class PathPlanner
  {
  public:


    void setInputMesh(vtkSmartPointer<vtkPolyData> mesh);

    vtkSmartPointer<vtkPolyData> getInputMesh(){return input_mesh_;}

    void setTool(ProcessTool tool){tool_ = tool;}

    ProcessTool getTool(){return tool_;}

    // Uses the input mesh, generates the first path by intersecting the mesh with a plane
    void getFirstPath(ProcessPath& path);

    // Given a path and distance, create the next offset path
    bool getNextPath(const ProcessPath this_path, ProcessPath& next_path, double dist);

    void computePaths();

    std::vector<ProcessPath> getPaths(){return paths_;}

  private:
    // mesh to operate on
    vtkSmartPointer<vtkPolyData> input_mesh_;

    // series of intersecting lines on the given mesh
    std::vector<ProcessPath> paths_;

    // The tool parameters which defines how to generate the tool paths (spacing, offset, etc.)
    ProcessTool tool_;

    // Takes in a series of points, performs spline interpolation
    void smoothData(vtkSmartPointer<vtkParametricSpline> spline, vtkSmartPointer<vtkPolyData>& points, vtkSmartPointer<vtkPolyData>& derivatives);

    // Generates point normals for an input mesh, normals are located on polydata vertices
    void generateNormals(vtkSmartPointer<vtkPolyData>& data);

    // For a set of new points, estimates the normal from the input mesh normals by averaging N nearest neighbors
    void estimateNewNormals(vtkSmartPointer<vtkPolyData>& data);

    // Given a line with normals, generate a new line which is offset by a given distance
    vtkSmartPointer<vtkPolyData> createOffsetLine(vtkSmartPointer<vtkPolyData> line, vtkSmartPointer<vtkPolyData> derivatives, double dist);

    // Using a line with normals, generate a surface with points above and below the line (used for mesh intersection calculation)
    vtkSmartPointer<vtkPolyData> createSurfaceFromSpline(vtkSmartPointer<vtkPolyData> line, double dist);

    void sortPoints(vtkSmartPointer<vtkPoints>& points);

    void flipPointOrder(ProcessPath& path);

    bool findIntersectionLine(vtkSmartPointer<vtkPolyData> cut_surface,
                                                      vtkSmartPointer<vtkPolyData>& points,
                                                      vtkSmartPointer<vtkParametricSpline>& spline);

    int findClosestPoint(std::vector<double>& pt,  std::vector<std::vector<double> >& pts);

    double pt_dist(double* pt1, double* pt2);

    double dist(std::vector<double>& pt1, std::vector<double>& pt2);


  };
}

#endif // PATH_PLANNER_H
