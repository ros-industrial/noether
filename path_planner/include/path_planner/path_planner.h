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

namespace path_planner
{
  class PathPlanner
  {
  public:
    void setInputMesh(vtkSmartPointer<vtkPolyData> mesh){_input_mesh = mesh;}

    // Uses the input mesh, generates the first path by intersecting the mesh with a plane
    vtkSmartPointer<vtkPolyData> getFirstPath();

    // Takes in a series of points, performs spline interpolation
    vtkSmartPointer<vtkPolyData> smoothData(vtkSmartPointer<vtkPoints> points, vtkSmartPointer<vtkPolyData>& derivatives);

    // Generates point normals for an input mesh, normals are located on polydata vertices
    vtkSmartPointer<vtkPolyData> generateNormals(vtkSmartPointer<vtkPolyData> data);

    // For a set of new points, estimates the normal from the input mesh normals by averaging N nearest neighbors
    void estimateNewNormals(vtkSmartPointer<vtkPolyData>& data);

    // Given a line with normals, generate a new line which is offset by a given distance
    vtkSmartPointer<vtkPolyData> createOffsetLine(vtkSmartPointer<vtkPolyData> line, vtkSmartPointer<vtkPolyData> derivatives, double dist);

    // Using a line with normals, generate a surface with points above and below the line (used for mesh intersection calculation)
    vtkSmartPointer<vtkPolyData> createSurfaceFromSpline(vtkSmartPointer<vtkPolyData> line, double dist);

  private:
    vtkSmartPointer<vtkPolyData> _input_mesh;

    vtkSmartPointer<vtkPoints> sortPoints(vtkSmartPointer<vtkPoints>& points);

    int findClosestPoint(std::vector<double>& pt,  std::vector<std::vector<double> >& pts);

    double dist(std::vector<double>& pt1, std::vector<double>& pt2);


  };
}

#endif // PATH_PLANNER_H
