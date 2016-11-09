/*
 * Copyright (c) 2016, Southwest Research Institute
 * All rights reserved.
 *
 */

#include "vtk_viewer/vtk_utils.h"

#include <sstream>

#include <vtkPointData.h>
#include <vtkMath.h>

#include <vtkTransform.h>
#include <vtkVersion.h>
#include <vtkProperty.h>
#include <vtkDelaunay2D.h>
#include <vtkPolyDataMapper.h>
#include <vtkActor.h>
#include <vtkRenderWindow.h>
#include <vtkRenderer.h>
#include <vtkRenderWindowInteractor.h>
#include <vtkVertexGlyphFilter.h>

namespace vtk_viewer
{


vtkSmartPointer<vtkPoints> createPlane()
{
  // Create points on an XY grid with random Z coordinate
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();

  unsigned int gridSize = 10;
  for(unsigned int x = 0; x < gridSize; x++)
    {
    for(unsigned int y = 0; y < gridSize; y++)
      {
      points->InsertNextPoint(x, y, 2.0 * cos(double(x)/2.0) - 2.0 * sin(double(y)/2.0) + vtkMath::Random(0.0, 0.1));
      }
    }

  // Add the grid points to a polydata object
  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
  polydata->SetPoints(points);

  return points;
}

vtkSmartPointer<vtkPolyData> createMesh(vtkSmartPointer<vtkPoints> points )
{
  if(!points)
  {
    points = createPlane();
  }

  vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
  polydata->SetPoints(points);

  vtkSmartPointer<vtkDelaunay2D> delaunay = vtkSmartPointer<vtkDelaunay2D>::New();
  delaunay->SetInputData(polydata);

  // find the bounds of the data, use this to set the transform
  points->ComputeBounds();
  double* bounds = points->GetBounds();  // xmin,xmax,ymin,ymax,zmin,zmax
  double x = fabs(bounds[0]- bounds[1]);
  double y = fabs(bounds[2]- bounds[3]);
  double z = fabs(bounds[4]- bounds[5]);

  double m = sqrt(pow(x,2.0) + pow(y,2.0) + pow(z,2.0));

  x = m/x;
  y = m/y;
  z = m/z;

  m = sqrt(pow(x,2.0) + pow(y,2.0) + pow(z,2.0));
  x = x/m;
  y = y/m;
  z = z/m;

  //cout << "normal direction : " << x << " " << y << " " << z << "\n";

  vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
  transform->PostMultiply();

  double theta = atan2(y,x);
  //double s = y/x < 1.0 ? y/x : y/x - 1.0;
  double s = y/x;
  double phi = asin(s);

  //cout << "theta/phi : " << theta << " " << phi  << "\n";

  // TODO: verify that the rotation for performing Delaunay triangulation is correct
  transform->RotateZ(theta );
  transform->RotateX(phi );

  delaunay->SetTransform(transform);

  // Update and return the results
  delaunay->Update();

  return delaunay->GetOutput();

}

void visualizePlane(vtkSmartPointer<vtkPolyData> &polydata)
{

  // Create a mapper and actor
  vtkSmartPointer<vtkPolyDataMapper> triangulatedMapper = vtkSmartPointer<vtkPolyDataMapper>::New();
  triangulatedMapper->SetInputData(polydata);

  vtkSmartPointer<vtkActor> triangulatedActor = vtkSmartPointer<vtkActor>::New();
  triangulatedActor->SetMapper(triangulatedMapper);

  // Create a renderer, render window, and interactor
  vtkSmartPointer<vtkRenderer> renderer = vtkSmartPointer<vtkRenderer>::New();
  vtkSmartPointer<vtkRenderWindow> renderWindow = vtkSmartPointer<vtkRenderWindow>::New();
  renderWindow->AddRenderer(renderer);
  vtkSmartPointer<vtkRenderWindowInteractor> renderWindowInteractor =
    vtkSmartPointer<vtkRenderWindowInteractor>::New();
  renderWindowInteractor->SetRenderWindow(renderWindow);

  // Add the actor to the scene

  renderer->AddActor(triangulatedActor);
  renderer->SetBackground(.3, .6, .3); // Background color green

  // Render and interact
  renderWindow->Render();
  renderWindowInteractor->Start();
}

}
