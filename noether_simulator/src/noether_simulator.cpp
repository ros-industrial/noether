/*
 * Copyright (c) 2016, Southwest Research Institute
 * All rights reserved.
 *
 */

#include <noether_simulator/noether_simulator.h>

#include <Eigen/Geometry>
#include <vtkCylinderSource.h>
#include <vtkDoubleArray.h>

#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>

#include <vtkDelaunay3D.h>
#include <vtkUnstructuredGrid.h>
#include <vtkGeometryFilter.h>

#include <vtkExtractPoints.h>
#include <vtkSelectEnclosedPoints.h>
#include <vtkModifiedBSPTree.h>
#include <vtkLookupTable.h>

namespace noether_simulator
{
  NoetherSimulator::NoetherSimulator()
  {
    process_base_rate_ = 5.0;
    scalar_sigma_ = 3.0;
    debug_on_ = false;
    simulation_points_ = vtkSmartPointer<vtkPolyData>::New();
  }

  double NoetherSimulator::integral(double pt[3])
  {
    //double k = process_base_rate_;
    double result;
    // t is not currently added/used, but can be so as to allow time parameterization between process points
    // f(x,y,z,t) = -x^2 - y^2 - (z - k)
    // integral(f) = -x^3*y*z/3 - x*y^3*z/3 - x*y*z^2/2 + k*x*y*z
    // TODO: the above equation does not work (integrates area, not along a line)
    // TODO: again, cylinder is along y-axis, so y and z are flipped
      result = -( pow(pt[0],3.0)) /3.0 - (pow(pt[2],3.0))/3.0 ;

    return 10*result;
  }

  double NoetherSimulator::calculateIntegration(vtkSmartPointer<vtkPoints> points)
  {
    if(points->GetNumberOfPoints() < 2)
      return 0;
    // This is a place holder equation for the tool and is process specific
    // TODO: create a more generic interface to allow for other tools to be created and used

    double pt1[3], pt2[3];
    double val1, val2;

    points->GetPoint(0, pt1);
    points->GetPoint(1, pt2);

    val1 = integral(pt1);
    val2 = integral(pt2);

    double value = (val2 - val1);
    return fabs(value);
  }

vtkSmartPointer<vtkPolyData>  NoetherSimulator::getSimulatedPoints()
  {
    vtkSmartPointer<vtkPolyData> simulated_points = simulation_points_;
    return simulated_points;
  }

  void  NoetherSimulator::runSimulation()
  {

    cout << "starting simulation\n";
    std::vector<float> color(3);
    color[0] = 0.9; color[1] = 0.9; color[2] = 0.9;

    // densly sample surface based upon point spacing
    double spacing = tool_.pt_spacing /10.0;
    vtkSmartPointer<vtkPolyData> simulation_points;
    simulation_points = vtk_viewer::sampleMesh(input_mesh_, spacing);

    // add scalar data for all points
    vtkSmartPointer<vtkDoubleArray> scalars = vtkSmartPointer<vtkDoubleArray>::New();
    scalars->SetNumberOfComponents(1);
    scalars->SetNumberOfTuples(simulation_points->GetNumberOfPoints());
    int number_Tuples = scalars->GetNumberOfTuples();
    for(int i = 0; i < number_Tuples; ++i)
    {
      scalars->SetTuple1(i, 0.0);
    }

    // create tool object
    vtkSmartPointer<vtkCylinderSource> cylinder = vtkSmartPointer<vtkCylinderSource>::New();
    cylinder->SetCenter(0.0, 0.0, 0.0);
    cylinder->SetRadius(tool_.tool_radius );
    cylinder->SetHeight(tool_.tool_height);
    cylinder->SetResolution(20);
    cylinder->Update();

    //vtkSmartPointer<vtkOBBTree> tree = vtkSmartPointer<vtkOBBTree>::New();
    vtkSmartPointer<vtkModifiedBSPTree> tree = vtkSmartPointer<vtkModifiedBSPTree>::New();
    tree->SetDataSet(cylinder->GetOutput());
    tree->BuildLocator();

    // use tool to integrate over time and distance
    int number_paths =input_paths_.size();
    for(int i = 0; i <number_paths; ++i)
    {
      cout << "simulating path " << i+1 << " of " << input_paths_.size() << "\n";

      tool_path_planner::ProcessPath this_path = input_paths_[i];
      int number_points = this_path.line->GetNumberOfPoints() - 1;
      for(int j = 0; j < number_points; ++j)
      {
        // get pt1 and pt2
        double path_pt1[3], norm1[3], derv1[3];
        this_path.line->GetPoints()->GetPoint(j, path_pt1);
        this_path.line->GetPointData()->GetNormals()->GetTuple(j, norm1);
        this_path.derivatives->GetPointData()->GetNormals()->GetTuple(j, derv1);

        double path_pt2[3], norm2[3], derv2[3];
        this_path.line->GetPoints()->GetPoint(j+1, path_pt2);
        this_path.line->GetPointData()->GetNormals()->GetTuple(j+1, norm2);
        this_path.derivatives->GetPointData()->GetNormals()->GetTuple(j+1, derv2);

        // transform tool to pt1 and pt2 location
        vtkSmartPointer<vtkMatrix4x4> transform1 = createMatrix(path_pt1, norm1, derv1);
        vtkSmartPointer<vtkMatrix4x4> transform2 = createMatrix(path_pt2, norm2, derv2);

        vtkSmartPointer<vtkTransform> trans = vtkSmartPointer<vtkTransform>::New();
        trans->SetMatrix(transform1);

        // transform to point 1 and get data
        vtkSmartPointer<vtkTransformPolyDataFilter> transform_filter =
          vtkSmartPointer<vtkTransformPolyDataFilter>::New();
        vtkSmartPointer<vtkPolyData> cylinder_poly = vtkSmartPointer<vtkPolyData>::New();
        cylinder_poly->DeepCopy(cylinder->GetOutput());
        transform_filter->SetInputData(cylinder_poly);
        transform_filter->SetTransform(trans);
        transform_filter->Update();

        vtkSmartPointer<vtkPolyData> point_set1 = vtkSmartPointer<vtkPolyData>::New();
        point_set1->DeepCopy(transform_filter->GetOutput());

        // transform to point 2 and get data
        trans->SetMatrix(transform2);
        transform_filter->SetTransform(trans);
        transform_filter->Update();

        vtkSmartPointer<vtkPolyData> point_set2 = vtkSmartPointer<vtkPolyData>::New();
        point_set2->DeepCopy(transform_filter->GetOutput());

        // create convex hull of two tool locations
        vtkSmartPointer<vtkDelaunay3D> delaunay_3D =
            vtkSmartPointer<vtkDelaunay3D>::New();
        vtkSmartPointer<vtkPolyData> hull_data = vtkSmartPointer<vtkPolyData>::New();
        vtkSmartPointer<vtkPoints> point_data = vtkSmartPointer<vtkPoints>::New();
        point_data->DeepCopy(point_set1->GetPoints());
        point_data->Resize(point_set1->GetPoints()->GetNumberOfPoints() + point_set2->GetPoints()->GetNumberOfPoints());
        point_data->InsertPoints(point_set1->GetPoints()->GetNumberOfPoints(), point_set2->GetPoints()->GetNumberOfPoints(), 0, point_set2->GetPoints());

        hull_data->SetPoints(point_data);

        delaunay_3D->SetInputData (hull_data);
        delaunay_3D->Update();

        vtkGeometryFilter *extract = vtkGeometryFilter::New();
        extract->SetInputData(delaunay_3D->GetOutput());
        extract->Update();

        vtkSmartPointer<vtkPolyData> convex_hull = vtkSmartPointer<vtkPolyData>::New();
        convex_hull = extract->GetOutput();

        // find all simulation points located inside convex hull
        vtkSmartPointer<vtkSelectEnclosedPoints> select_enclosed =
            vtkSmartPointer<vtkSelectEnclosedPoints>::New();
        select_enclosed->SetTolerance(0.0001);
        select_enclosed->SetInputData(simulation_points);
        select_enclosed->SetSurfaceData(convex_hull);
        select_enclosed->Update();

        int num_pts = simulation_points->GetPoints()->GetNumberOfPoints();
        vtkSmartPointer<vtkIdList> pts_inside = vtkSmartPointer<vtkIdList>::New();
        for(int i = 0; i < num_pts ; ++i)
        {
          if(select_enclosed->IsInside(i))
          {
            pts_inside->InsertNextId(i);
          }
        }
        vtkSmartPointer<vtkPoints> tool_points = vtkSmartPointer<vtkPoints>::New();
        simulation_points->GetPoints()->GetPoints(pts_inside, tool_points);

        // transform all points into tool_pt1 frame of referece
        vtkSmartPointer<vtkPolyData> temp_data = vtkSmartPointer<vtkPolyData>::New();
        temp_data->SetPoints(tool_points);
        vtkSmartPointer<vtkPoints> tool_pts1 = vtkSmartPointer<vtkPoints>::New();
        trans->SetMatrix(transform1);
        trans->Inverse();
        transform_filter->SetInputData(temp_data);
        transform_filter->SetTransform(trans);
        transform_filter->Update();
        tool_pts1->DeepCopy(transform_filter->GetOutput()->GetPoints());

        // transform all points into tool_pt2 frame of referece
        vtkSmartPointer<vtkPoints> tool_pts2 = vtkSmartPointer<vtkPoints>::New();
        trans->SetMatrix(transform2);
        trans->Inverse();
        transform_filter->SetTransform(trans);
        transform_filter->Update();
        tool_pts2->DeepCopy(transform_filter->GetOutput()->GetPoints());

        // use tool_pt1 and tool_pt2 to calculate deposition for each point
        // using tool deposition model.
        vtkSmartPointer<vtkSelectEnclosedPoints> select_enclosed2 =
            vtkSmartPointer<vtkSelectEnclosedPoints>::New();
        select_enclosed2->SetTolerance(0.000001);
        num_pts = tool_points->GetNumberOfPoints();
        select_enclosed2->SetSurfaceData(cylinder->GetOutput());
        vtkSmartPointer<vtkPolyData> test_poly = vtkSmartPointer<vtkPolyData>::New();
        vtkSmartPointer<vtkPoints> test_pts = vtkSmartPointer<vtkPoints>::New();
        vtkSmartPointer<vtkPoints> integration_pts = vtkSmartPointer<vtkPoints>::New();

        //test_pts->SetNumberOfPoints(2);
        for(int i = 0; i < num_pts; ++i)
        {
          integration_pts->Reset();

          // check to see if the point begin/end locations are inside of the tool
          double pt1[3], pt2[3];
          tool_pts1->GetPoint(i, pt1);
          tool_pts2->GetPoint(i, pt2);

          test_pts->Reset();
          test_pts->InsertNextPoint(pt1);
          test_pts->InsertNextPoint(pt2);
          test_poly->SetPoints(test_pts);
          select_enclosed2->SetInputData(test_poly);
          select_enclosed2->Update();

          if(select_enclosed2->IsInside(0))
          {
            integration_pts->InsertNextPoint(pt1);
          }
          if(select_enclosed2->IsInside(1))
          {
            integration_pts->InsertNextPoint(pt2);
          }

          // if one or both points are not inside the tool,
          // create line from point start/stop location and find intersection with the tool
          if(integration_pts->GetNumberOfPoints() < 2)
          {
            vtkSmartPointer<vtkPoints> temp_pts = vtkSmartPointer<vtkPoints>::New();
            tree->IntersectWithLine(pt1, pt2, 0.001, temp_pts, NULL);

            int intersections = temp_pts->GetNumberOfPoints();

            // get intersection point(s)
            for(int j = 0; j < intersections; ++j)
            {
              double temp_pt[3];
              temp_pts->GetPoint(j, temp_pt);
              integration_pts->InsertNextPoint(temp_pt);
            }
          }

          // We should now have two points; combine pts with tool to get integrated value
          double integral_sum = 0;
          integral_sum = calculateIntegration(integration_pts);

          // assuming that the tool_pts are in the same order as pts_inside,
          // use the point indeces from pts_inside to modify the scalar values
          int index = pts_inside->GetId(i);
          // get scalar value, color data is stored in scalars
          double *tmp_value = scalars->GetTuple(index);
          double value = *tmp_value + integral_sum;
          scalars->SetTuple1(index, value);

        }// end loop through all tool points found
      }// end loop through all process points in this path

    }// end loop through all paths

    // display of simulation points

    // create look up table for point coloring
    simulation_points->GetPointData()->SetScalars(scalars);

    simulation_points_->Reset();
    simulation_points_ = simulation_points;

    displayResults();

  } // end runSimulation()

  void NoetherSimulator::displayResults()
  {
    if(!simulation_points_)
    {
      return;
    }

    double mean = 0;
    double *value;
    vtkSmartPointer<vtkDoubleArray> scalars = vtkSmartPointer<vtkDoubleArray>::New();
    scalars =  vtkDoubleArray::SafeDownCast(simulation_points_->GetPointData()->GetScalars());

    // calculate mean of data and std_dev of data
    int number_tuples = scalars->GetNumberOfTuples();
    for(int i = 0; i < number_tuples; ++i)
    {
      value = scalars->GetTuple(i);
      mean += *value;
    }
    mean = mean/scalars->GetNumberOfTuples();
    double std_dev = 0;
    int number_scalars = scalars->GetNumberOfTuples();
    for(int i = 0; i < number_scalars; ++i)
    {
      value = scalars->GetTuple(i);
      std_dev += pow((mean - *value), 2.0);
    }
    std_dev = std_dev/ (scalars->GetNumberOfTuples() - 1);
    std_dev = sqrt(std_dev);

    // calculate the lower and upper limits for the display color range
    double range[2];
    scalars->GetRange(range);

    cout << "scalar range " << range[0] << " " << range[1] << "\n";
    double upper_bound = mean + std_dev * scalar_sigma_;
    double lower_bound = mean - std_dev * scalar_sigma_;

    // TODO: this limit assumes that the process is an additive one,
    //       for subtractive processes, something else will need to be done
    if(lower_bound < 0.0)
    {
      lower_bound = 0.0;
    }

    vtk_viewer::VTKViewer viewer;
    std::vector<float> color2(3);
    color2[0] = 0.1; color2[1] = 0.1; color2[2] = 0.1;
    viewer.addPolyDataDisplay(simulation_points_, color2, lower_bound, upper_bound);
    viewer.renderDisplay();
  }

  vtkSmartPointer<vtkMatrix4x4> NoetherSimulator::createMatrix(double pt[3], double norm[3], double derv[3])
  {
    // Get the point normal and derivative for creating the 3x3 transform
    //double* norm = paths[j].line->GetPointData()->GetNormals()->GetTuple(k);
    //double* der = paths[j].derivatives->GetPointData()->GetNormals()->GetTuple(k);

    // perform cross product to get the third axis direction
    Eigen::Vector3d u(norm[0], norm[1], norm[2]);
    Eigen::Vector3d v(derv[0], derv[1], derv[2]);
    Eigen::Vector3d w = u.cross(v);
    w.normalize();

    // after first cross product, u and w will be orthogonal.  Perform cross
    // product one more time to make sure that v is perfectly orthogonal to u and w
    v = u.cross(w);
    v.normalize();

    // TODO: the test currently uses a cylinder source, with the y-axis as the major axis
    // for correct visualization, need to set the norm vector (u) to the y-axis
    Eigen::Affine3d epose = Eigen::Affine3d::Identity();
    epose.matrix().col(0).head<3>() = w;
    epose.matrix().col(1).head<3>() = u;
    epose.matrix().col(2).head<3>() = -v;
    epose.matrix().col(3).head<3>() = Eigen::Vector3d(pt[0], pt[1], pt[2]);

    // the eigen epose.data() returns column major data whereas the vtk matrix DeepCopy()
    // takes row major data, need to transpose the data before setting the vtk matrix
    vtkSmartPointer<vtkMatrix4x4> temp_matrix = vtkSmartPointer<vtkMatrix4x4>::New();
    double* array = epose.data();
    double out_array[16];
    out_array[0] = array[0]; out_array[1] = array[4]; out_array[2] = array[8]; out_array[3] = array[12];
    out_array[4] = array[1]; out_array[5] = array[5]; out_array[6] = array[9]; out_array[7] = array[13];
    out_array[8] = array[2]; out_array[9] = array[6]; out_array[10] = array[10]; out_array[11] = array[14];
    out_array[12] = array[3]; out_array[13] = array[7]; out_array[14] = array[11]; out_array[15] = array[15];

    temp_matrix->DeepCopy(out_array);
    return temp_matrix;
  }

}
