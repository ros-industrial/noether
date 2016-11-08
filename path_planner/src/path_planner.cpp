/*
 * Copyright (c) 2016, Southwest Research Institute
 * All rights reserved.
 *
 */

#include <path_planner/path_planner.h>
#include <limits>

#include <vtkIntersectionPolyDataFilter.h>
#include <vtkDelaunay2D.h>
#include <vtkMath.h>
#include <vtkParametricSpline.h>
#include <vtkParametricFunctionSource.h>
#include <vtkCellLinks.h>
#include <vtkPolyDataNormals.h>

#include <vtkDoubleArray.h>
#include <vtkCellData.h>
#include <vtkPointData.h>

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

    // use Intersection filter
    vtkSmartPointer<vtkIntersectionPolyDataFilter> intersectionPolyDataFilter =
      vtkSmartPointer<vtkIntersectionPolyDataFilter>::New();
    intersectionPolyDataFilter->SetInputData( 0, _input_mesh);
    intersectionPolyDataFilter->SetInputData( 1, polydata2 );
    intersectionPolyDataFilter->Update();

    // return results
    return intersectionPolyDataFilter->GetOutput();

  }

  vtkSmartPointer<vtkPolyData> PathPlanner::smoothData(vtkSmartPointer<vtkPoints> points)
  {
    vtkSmartPointer<vtkPoints> points2 = vtkSmartPointer<vtkPoints>::New();

    points2 = sortPoints(points);

    vtkSmartPointer<vtkParametricSpline> spline = vtkSmartPointer<vtkParametricSpline>::New();

    //spline->ParameterizeByLengthOn();
    //spline->SetParameterizeByLength(1);

    spline->SetPoints(points2);

    vtkSmartPointer<vtkParametricFunctionSource> functionSource =
      vtkSmartPointer<vtkParametricFunctionSource>::New();
    functionSource->SetParametricFunction(spline);

    functionSource->SetScalarModeToDistance();
    functionSource->Update();

    vtkSmartPointer<vtkPolyData> output = vtkSmartPointer<vtkPolyData>::New();
    output = functionSource->GetOutput();

    vtkSmartPointer<vtkPoints> points3 = vtkSmartPointer<vtkPoints>::New();

    //int max = output->GetPoints()->GetNumberOfPoints();
    // get points which are evenly spaced along the spline
    int max = 10;
    double u[3], pt[3], d[9];
    for( int i = 0; i <= max; ++i)
    {

      u[0] = double(i)/double(max);
      u[1] = double(i)/double(max);
      u[2] = double(i)/double(max);
      spline->Evaluate(u, pt, d);
      points3->InsertNextPoint(pt);
    }
    output->SetPoints(points3);

    //return functionSource->GetOutput();
    return output;
  }

  // Sort points in linear order
  vtkSmartPointer<vtkPoints> PathPlanner::sortPoints(vtkSmartPointer<vtkPoints>& points)
  {
    std::vector<std::vector<double> > new_points;
    double d[3];
    for(int i = 0; i < points->GetNumberOfPoints(); ++i)
    {
        points->GetPoint(i, d);
        std::vector<double> pt;
        pt.push_back(d[0]);
        pt.push_back(d[1]);
        pt.push_back(d[2]);
        new_points.push_back(pt);
    }

    std::vector<std::vector<double> > sorted_points;

    // create new vector of sorted points
    int size = new_points.size();
    while(sorted_points.size() != size)
    {
      if(sorted_points.size() == 0)
      {
        sorted_points.push_back(new_points.front());
        new_points.erase(new_points.begin());
      }
      else
      {
        int next = findClosestPoint(sorted_points.back(), new_points);
        if (dist(sorted_points.back(), new_points[next]) < dist(sorted_points.front(), new_points[next]))
        {
          sorted_points.push_back(new_points[next]);
          new_points.erase(new_points.begin() + next);
        }
        else
        {
          next = findClosestPoint(sorted_points.front(), new_points);
          sorted_points.insert(sorted_points.begin(), new_points[next]);
          new_points.erase(new_points.begin() + next);
        }
      }
    }

    // convert array to vtkPoints and return
    vtkSmartPointer<vtkPoints> line_points = vtkSmartPointer<vtkPoints>::New();
    for(int i = 0; i < size; ++i)
    {
      line_points->InsertNextPoint(sorted_points[i][0], sorted_points[i][1], sorted_points[i][2]);
    }

    return line_points;
  }

  int PathPlanner::findClosestPoint(std::vector<double>& pt,  std::vector<std::vector<double> >& pts)
  {
    double min = std::numeric_limits<double>::max();
    int index = -1;
    for(int i = 0; i < pts.size(); ++i)
    {
      double d = dist(pt, pts[i]);
      if(d < min)
      {
        index = i;
        min = d;
      }
    }
    return index;
  }

  double PathPlanner::dist(std::vector<double>& pt1, std::vector<double>& pt2)
  {
    return (pow(pt1[0] - pt2[0], 2) + pow(pt1[1] - pt2[1], 2 ) + pow((pt1[2] - pt2[2]), 2 ));
  }

  vtkSmartPointer<vtkPolyData> PathPlanner::generateNormals(vtkSmartPointer<vtkPolyData> data)
  {

    vtkSmartPointer<vtkPolyData> polydata = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPolyDataNormals> normalGenerator = vtkSmartPointer<vtkPolyDataNormals>::New();
    normalGenerator->SetInputData(data);
    normalGenerator->ComputePointNormalsOn();
    normalGenerator->ComputeCellNormalsOff();

    // Optional settings
    normalGenerator->SetFeatureAngle(0.1);
    normalGenerator->SetSplitting(0);
    normalGenerator->SetConsistency(1);
    normalGenerator->SetAutoOrientNormals(0);
    normalGenerator->SetComputePointNormals(1);
    normalGenerator->SetComputeCellNormals(0);
    normalGenerator->SetFlipNormals(1);
    normalGenerator->SetNonManifoldTraversal(1);

    normalGenerator->Update();

    polydata = normalGenerator->GetOutput();

    vtkDataArray* normalsGeneric = polydata->GetPointData()->GetNormals(); //works
      if(normalsGeneric)
      {
        cout << "normals exist" ;
      }

//    vtkSmartPointer<vtkDoubleArray> pointNormalsRetrieved =
//        vtkDoubleArray::SafeDownCast(polydata->GetPolys()->GetData());
//      if(pointNormalsRetrieved)
//        {
//        std::cout << "There are " << pointNormalsRetrieved->GetNumberOfTuples()
//                  << " point normals." << std::endl;

//      }

    return polydata;

  }
}
