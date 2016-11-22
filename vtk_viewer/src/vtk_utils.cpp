/*
 * Copyright (c) 2016, Southwest Research Institute
 * All rights reserved.
 *
 */

#include "vtk_viewer/vtk_utils.h"

#include <sstream>

#include <vtkPointData.h>
#include <vtkCellData.h>
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
#include <vtkSTLReader.h>

#include <vtkPolyDataNormals.h>
#include <vtkDoubleArray.h>
#include <vtkMarchingContourFilter.h>
#include <vtkMarchingCubes.h>
#include <vtkCurvatures.h>

#include <vtkSurfaceReconstructionFilter.h>
#include <vtkReverseSense.h>
#include <vtkContourFilter.h>
#include <vtkPolyDataPointSampler.h>



namespace vtk_viewer
{


vtkSmartPointer<vtkPoints> createPlane()
{
  // Create points on an XY grid with a sinusoidal Z component
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();

  unsigned int gridSize = 10;
  for(unsigned int x = 0; x < gridSize; x++)
    {
    for(unsigned int y = 0; y < gridSize; y++)
      {
      points->InsertNextPoint(x, y, 1.0 * cos(double(x)/2.0) - 1.0 * sin(double(y)/2.0) + vtkMath::Random(0.0, 0.1));
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

  // In order to perform the Delaunay2D triangulation, all of the points must be roughly
  // in the x/y plane (z component is ignored).  If they are otherwise, the triangulation results will
  // not be ideal and result in spurious or ill formed triangles.  Need to find a rotation
  // to place the points in the desired x/y plane

  vtkSmartPointer<vtkTransform> transform = vtkSmartPointer<vtkTransform>::New();
  transform->PostMultiply();

  // find the bounds of the data, use this to set the transform
  points->ComputeBounds();
  double* bounds = points->GetBounds();  // xmin,xmax,ymin,ymax,zmin,zmax
  double x = 1.0/fabs(bounds[0]- bounds[1]);
  double y = 1.0/fabs(bounds[2]- bounds[3]);
  double z = 1.0/fabs(bounds[4]- bounds[5]);

  // normalize the x,y,z components using the sqrt of the sum of the squares
  double m = sqrt(pow(x,2.0) + pow(y,2.0) + pow(z,2.0));
  x = x/m;
  y = y/m;
  z = z/m;

  // calculate the rotation to place the data in the x/y plane
  double theta = atan2(y,x);

  // TODO: verify that the rotation for performing Delaunay triangulation is correct
  // May need to perform one more rotation
  transform->RotateZ(theta );
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

vtkSmartPointer<vtkPolyData> readSTLFile(std::string file)
{
  vtkSmartPointer<vtkSTLReader> reader = vtkSmartPointer<vtkSTLReader>::New();
  reader->SetFileName(file.c_str());
  reader->SetMerging(1);
  reader->Update();

  return reader->GetOutput();
}



vtkSmartPointer<vtkPolyData> estimateCurvature(vtkSmartPointer<vtkPolyData> mesh, int method)
{
  vtkSmartPointer<vtkCurvatures> curvaturesFilter =
      vtkSmartPointer<vtkCurvatures>::New();
    curvaturesFilter->SetInputData(mesh);
    curvaturesFilter->SetCurvatureTypeToMean();
    switch(method)
    {
    case 0:
      curvaturesFilter->SetCurvatureTypeToMinimum();
      break;
    case 1:
      curvaturesFilter->SetCurvatureTypeToMaximum();
      break;
    case 2:
      curvaturesFilter->SetCurvatureTypeToGaussian();
      break;
    case 3:
      curvaturesFilter->SetCurvatureTypeToMean();
      break;
    }
    curvaturesFilter->Update();

    return curvaturesFilter->GetOutput();
}

void generateNormals(vtkSmartPointer<vtkPolyData>& data)
{
  vtkSmartPointer<vtkPolyDataNormals> normalGenerator = vtkSmartPointer<vtkPolyDataNormals>::New();
  normalGenerator->SetInputData(data);
  normalGenerator->ComputePointNormalsOn();
  normalGenerator->ComputeCellNormalsOff();

  // Optional settings
  normalGenerator->SetFeatureAngle(0.1);
  normalGenerator->SetSplitting(0);
  normalGenerator->SetConsistency(1);
  normalGenerator->SetAutoOrientNormals(1);
  normalGenerator->SetComputePointNormals(1);
  normalGenerator->SetComputeCellNormals(1);
  normalGenerator->SetFlipNormals(0);
  normalGenerator->SetNonManifoldTraversal(1);

  normalGenerator->Update();

  vtkDataArray* normals = normalGenerator->GetOutput()->GetPointData()->GetNormals();
  if(normals)
  {
    data->GetPointData()->SetNormals(normals);
  }

  vtkDataArray* normals2 = normalGenerator->GetOutput()->GetCellData()->GetNormals();
  if(normals2)
  {
    data->GetCellData()->SetNormals(normals2);
  }
}

vtkSmartPointer<vtkPolyData> upsampleMesh(vtkSmartPointer<vtkPolyData> mesh, double distance)
{
  vtkSmartPointer<vtkPolyDataPointSampler> pointSampler =
      vtkSmartPointer<vtkPolyDataPointSampler>::New();
  pointSampler->SetDistance(distance);
  pointSampler->SetInputData(mesh);
  pointSampler->Update();

  // Resize the mesh and insert the new points
  int old_size = mesh->GetPoints()->GetNumberOfPoints();
  int new_size = pointSampler->GetOutput()->GetPoints()->GetNumberOfPoints();

  vtkSmartPointer<vtkPolyData> updated_mesh = vtkSmartPointer<vtkPolyData>::New();
  updated_mesh->SetPoints(mesh->GetPoints());
  int size = old_size + new_size;
  updated_mesh->GetPoints()->Resize(size);

  //updated_mesh->GetPoints()->InsertPoints(0, old_size, 0, mesh->GetPoints());
  updated_mesh->GetPoints()->InsertPoints(old_size, new_size, 0, pointSampler->GetOutput()->GetPoints());

  // create surface from new point set
  vtkSmartPointer<vtkSurfaceReconstructionFilter> surf =
      vtkSmartPointer<vtkSurfaceReconstructionFilter>::New();

  surf->SetInputData(updated_mesh);
  cout << pointSampler->GetOutput()->GetPoints()->GetNumberOfPoints() << "\n";

  //vtkSmartPointer<vtkMarchingContourFilter> cf = vtkSmartPointer<vtkMarchingContourFilter>::New();
  vtkSmartPointer<vtkMarchingCubes> cf = vtkSmartPointer<vtkMarchingCubes>::New();

  cf->SetInputConnection(surf->GetOutputPort());
  cf->SetValue(0, 0.0);
  cf->ComputeNormalsOn();

  cf->Update();

  // Sometimes the contouring algorithm can create a volume whose gradient
  // vector and ordering of polygon (using the right hand rule) are
  // inconsistent. vtkReverseSense cures this problem.
  vtkSmartPointer<vtkReverseSense> reverse = vtkSmartPointer<vtkReverseSense>::New();
  reverse->SetInputConnection(cf->GetOutputPort());
  reverse->ReverseCellsOff();
  reverse->ReverseNormalsOff();
  reverse->Update();

  return reverse->GetOutput();
}



}
