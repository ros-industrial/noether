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

    vtkSmartPointer<vtkPolyData> getFirstPath();

    vtkSmartPointer<vtkPolyData> smoothData(vtkSmartPointer<vtkPoints> points);

    vtkSmartPointer<vtkPolyData> generateNormals(vtkSmartPointer<vtkPolyData> data);
  private:
    vtkSmartPointer<vtkPolyData> _input_mesh;

    vtkSmartPointer<vtkPoints> sortPoints(vtkSmartPointer<vtkPoints>& points);

    int findClosestPoint(std::vector<double>& pt,  std::vector<std::vector<double> >& pts);

    double dist(std::vector<double>& pt1, std::vector<double>& pt2);

  };
}

#endif // PATH_PLANNER_H
