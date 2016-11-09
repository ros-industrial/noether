/*
 * Copyright (c) 2016, Southwest Research Institute
 * All rights reserved.
 *
 */


#ifndef VTK_UTILS_H
#define VTK_UTILS_H

#include <vtkSmartPointer.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>

namespace vtk_viewer
{

vtkSmartPointer<vtkPoints> createPlane();

void visualizePlane(vtkSmartPointer<vtkPolyData>& polydata);

vtkSmartPointer<vtkPolyData> createMesh(vtkSmartPointer<vtkPoints> points);
}
#endif // VTK_UTILS_H
