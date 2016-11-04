/*
 * Copyright (c) 2016, Southwest Research Institute
 * All rights reserved.
 *
 */

#include <path_planner/path_planner.h>

#include <vtkIntersectionPolyDataFilter.h>
#include <vtkDelaunay2D.h>
#include <vtkMath.h>
#include <vtkParametricSpline.h>
#include <vtkParametricFunctionSource.h>
#include <vtkCellLinks.h>

namespace path_planner
{
  vtkSmartPointer<vtkPolyData> PathPlanner::getFirstPath()
  {
    // create vertical plane
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    unsigned int gridSize = 10;
    for(unsigned int x = 0; x < gridSize; x++)
      {
      for( int z = -5; z < 5; z++)
        {
        points->InsertNextPoint( x , (5 - z + vtkMath::Random(0.0, 0.1)) , z);
        }
      }

    vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPolyData> polydata2 = vtkSmartPointer<vtkPolyData>::New();
    polydata->SetPoints(points);
    vtkSmartPointer<vtkDelaunay2D> delaunay = vtkSmartPointer<vtkDelaunay2D>::New();
    delaunay->SetInputData(polydata);
    delaunay->Update();

    delaunay->SetAlpha(3);
    delaunay->SetOffset(3);
    //delaunay->SetProjectionPlaneMode(3);
    polydata2 = delaunay->GetOutput();


    cout << "number of input points : " << _input_mesh->GetPolys()->GetNumberOfCells() << "\n";
    cout << "number of plane points : " << polydata2->GetPolys()->GetNumberOfCells() << "\n";

    // use Intersection filter
    vtkSmartPointer<vtkIntersectionPolyDataFilter> intersectionPolyDataFilter =
      vtkSmartPointer<vtkIntersectionPolyDataFilter>::New();
    intersectionPolyDataFilter->SetInputData( 0, _input_mesh);
    intersectionPolyDataFilter->SetInputData( 1, polydata2 );
    intersectionPolyDataFilter->Update();

    // return results

    cout << "build links" << "\n";
    vtkSmartPointer<vtkCellLinks> cellLinksFilter =
        vtkSmartPointer<vtkCellLinks>::New();

    vtkSmartPointer<vtkPolyData> polylines;
    polylines = (intersectionPolyDataFilter->GetOutput());

    // Sort points using line data
    vtkSmartPointer<vtkPoints> line_points;
    vtkSmartPointer<vtkPoints> line_points2 = vtkSmartPointer<vtkPoints>::New();

    line_points = intersectionPolyDataFilter->GetOutput()->GetPoints();
    vtkSmartPointer<vtkCellArray> lines = polylines->GetLines();

    lines->InitTraversal();

    cout << "lines: " << polylines->GetNumberOfLines() << "\n";
    cout << "cells: " << polylines->GetNumberOfCells() << "\n";

    vtkSmartPointer<vtkIdList> pt = vtkSmartPointer<vtkIdList>::New();
    int cell = lines->GetNextCell(pt);
    int next_cell = 0;

    while(cell != 0)
    //for(int i = 0; i < polylines->GetNumberOfLines(); ++i)
    {
      lines->InitTraversal();
      for(int j = 0; j < polylines->GetNumberOfLines(); ++j)
      {
        cell = lines->GetNextCell(pt);
//        if(i == polylines->GetNumberOfLines() - 1)
//        {
//        cout << "  "  << pt->GetId(0) << " " << pt->GetId(1) << "\n";
//        }


        if(cell == 0)
        {
          next_cell += 2;
          break;
        }

        if(pt->GetId(0) == next_cell)
        {
          cout << " found cell " << next_cell << "   "  << pt->GetId(0) << " " << pt->GetId(1) << "\n";
          if(next_cell == 0)
          {
            double d[3];
            line_points->GetPoint(j, d);
            cout << "first point " << d[0] << " " << d[1] << " " << d[2] << "\n";
            line_points2->InsertNextPoint(d[0], d[1], d[2]);
          }
            double d[3];
            line_points->GetPoint(pt->GetId(1), d);
            cout << "next point " << d[0] << " " << d[1] << " " << d[2] << "\n";
            line_points2->InsertNextPoint(d[0], d[1], d[2]);
          next_cell = pt->GetId(1);
          break;
        }

      }
    }

    vtkSmartPointer<vtkPolyData> output_data = vtkSmartPointer<vtkPolyData>::New();
    output_data->SetPoints(line_points2);

    return output_data;
    //return intersectionPolyDataFilter->GetOutput();
    //return polydata2;
  }

  vtkSmartPointer<vtkPolyData> PathPlanner::smoothData(vtkSmartPointer<vtkPoints> points)
  {
    vtkSmartPointer<vtkParametricSpline> spline = vtkSmartPointer<vtkParametricSpline>::New();
      spline->SetPoints(points);

      vtkSmartPointer<vtkParametricFunctionSource> functionSource =
        vtkSmartPointer<vtkParametricFunctionSource>::New();
      functionSource->SetParametricFunction(spline);
      functionSource->Update();

    return functionSource->GetOutput();
  }

  // Sort points in linear order
  void PathPlanner::sortPoints(vtkSmartPointer<vtkPoints>& points)
  {

  }
}
