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

  private:
    vtkSmartPointer<vtkPolyData> _input_mesh;

    void sortPoints(vtkSmartPointer<vtkPoints>& points);
  };
}

#endif // PATH_PLANNER_H
