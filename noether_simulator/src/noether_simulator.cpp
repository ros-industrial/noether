/*
 * Software License Agreement (Apache License)
 *
 * Copyright (c) 2016, Southwest Research Institute
 *
 * file noether_simulator.cpp
 * All rights reserved.
 * copyright Copyright (c) 2019, Southwest Research Institute
 *
 * License
 * Software License Agreement (Apache License)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */
#include <ros/ros.h>
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

//Returns a copy of the private varible simulation_points
vtkSmartPointer<vtkPolyData>  NoetherSimulator::getSimulatedPoints()
  {
    vtkSmartPointer<vtkPolyData> simulated_points = simulation_points_;
    return simulated_points;
  }

//simulate the process on the mesh and path
void  NoetherSimulator::runSimulation()
  {
    std::vector<float> color(3);
    color[0] = 0.9; color[1] = 0.9; color[2] = 0.9;

    // densly sample surface based upon point spacing
    double spacing = tool_.pt_spacing /10.0;
    vtkSmartPointer<vtkPolyData> simulation_points;//output stored as private member variable
    simulation_points = vtk_viewer::sampleMesh(input_mesh_, spacing);

    // add scalar data for all points
    // where color information is stored
    vtkSmartPointer<vtkDoubleArray> scalars = vtkSmartPointer<vtkDoubleArray>::New();
    scalars->SetNumberOfComponents(1);
    scalars->SetNumberOfTuples(simulation_points->GetNumberOfPoints());
    int number_Tuples = scalars->GetNumberOfTuples();
    for(int i = 0; i < number_Tuples; ++i)
    {
      scalars->SetTuple1(i, 0.0);
    }

    // create tool object
    // the cylinder is defined in a different frame than the mesh or path
    vtkSmartPointer<vtkCylinderSource> cylinder = vtkSmartPointer<vtkCylinderSource>::New();
    cylinder->SetCenter(0.0, 0.0,0.0);
    cylinder->SetRadius(tool_.simulator_tool_radius);
    cylinder->SetHeight(tool_.simulator_tool_height);
    cylinder->SetResolution(20);
    cylinder->Update();

    //test cone
    vtkSmartPointer<vtkConeSource> cone = vtkSmartPointer<vtkConeSource>::New();
    cone->SetCenter(0.0, 0.0, (double(tool_.simulator_tool_height)/2.0));
    cone->SetRadius(tool_.simulator_tool_radius);
    double temp [3];
    cone->GetDirection(temp);
    cone->SetDirection(0,0,1);
    cone->GetDirection(temp);
    cone->SetHeight(tool_.simulator_tool_height);
    cone->SetResolution(20);
    cone->Update();
    vtkSmartPointer<vtkPolyData> cone_poly = vtkSmartPointer<vtkPolyData>::New();
    cone_poly->DeepCopy(cone->GetOutput());
    vtk_viewer::VTKViewer viewer;
    std::vector<float> color2(3);
    color2[0] = 0.1; color2[1] = 0.1; color2[2] = 0.1;
    //viewer.renderDisplay();
    //end test cone

    vtkSmartPointer<vtkModifiedBSPTree> tree = vtkSmartPointer<vtkModifiedBSPTree>::New();
    tree->SetDataSet(cylinder->GetOutput());
    tree->BuildLocator();

    // use tool to integrate over time and distance
    int number_paths = input_paths_.size();
    for(int i = 0; i <number_paths; ++i)
    {
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

        // transform tool (cylinder) to pt1 and pt2 location
        vtkSmartPointer<vtkMatrix4x4> transform1 = createMatrix(path_pt1, norm1, derv1);//set up to rotate cylinder to pt1 frame
        vtkSmartPointer<vtkMatrix4x4> transform2 = createMatrix(path_pt2, norm2, derv2);//set up to rotate cylinder to pt2 frame

        vtkSmartPointer<vtkTransform> trans = vtkSmartPointer<vtkTransform>::New();
        trans->SetMatrix(transform1);//transform tool to pt1 frame

        // transform to point 1 and get data
        vtkSmartPointer<vtkTransformPolyDataFilter> transform_filter =
          vtkSmartPointer<vtkTransformPolyDataFilter>::New();
        vtkSmartPointer<vtkPolyData> cylinder_poly = vtkSmartPointer<vtkPolyData>::New();
        //cylinder_poly->DeepCopy(cylinder->GetOutput());
        cylinder_poly->DeepCopy(cone->GetOutput());

        transform_filter->SetInputData(cylinder_poly);//set data to be transformed (tool)
        transform_filter->SetTransform(trans);//set transform (tramsform tool to pt1 frame)
        transform_filter->Update();

        vtkSmartPointer<vtkPolyData> point_set1 = vtkSmartPointer<vtkPolyData>::New();
        point_set1->DeepCopy(transform_filter->GetOutput());//holds all the points in the cylinder transformed to pt1 frame

        // transform to point 2 and get data
        trans->SetMatrix(transform2);//transform tool to pt2
        transform_filter->SetTransform(trans);
        transform_filter->Update();

        vtkSmartPointer<vtkPolyData> point_set2 = vtkSmartPointer<vtkPolyData>::New();
        point_set2->DeepCopy(transform_filter->GetOutput());//holds all the points in the cylinder transformed to pt2 frame

        // create convex hull of two tool locations
        vtkSmartPointer<vtkDelaunay3D> delaunay_3D =
            vtkSmartPointer<vtkDelaunay3D>::New();
        vtkSmartPointer<vtkPolyData> hull_data = vtkSmartPointer<vtkPolyData>::New();
        vtkSmartPointer<vtkPoints> point_data = vtkSmartPointer<vtkPoints>::New();
        point_data->DeepCopy(point_set1->GetPoints());//holds all the points in the cylinder transformed to pt1 frame
        point_data->Resize(point_set1->GetPoints()->GetNumberOfPoints() + point_set2->GetPoints()->GetNumberOfPoints());//resize to hold both pt1 and pt2 data
        point_data->InsertPoints(point_set1->GetPoints()->GetNumberOfPoints(), point_set2->GetPoints()->GetNumberOfPoints(), 0, point_set2->GetPoints());
        //adds the data stored in point_set2

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
        simulation_points->GetPoints()->GetPoints(pts_inside, tool_points);//pts_inside contains the points from the mesh inside the convex hull
        //tool_points contains the convex hull of the transformed tool.

        // transform all points into tool_pt1 frame of referece
        //transform both the convex hull and overlaying points from the mesh back to the tool frame of reference

        //transform all points into tool_pt1 frame of referece
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
       // vtkSmartPointer<vtkPoints> integration_pts = vtkSmartPointer<vtkPoints>::New();

        for(int i = 0; i < num_pts; ++i)//for each point inside convex hull incrament the scalor (color) value
        {
          // use the point indeces from pts_inside to modify the scalar values
          int index = pts_inside->GetId(i);
          // get scalar value, color data is stored in scalars
          double *tmp_value = scalars->GetTuple(index);
          //double value = *tmp_value + integral_sum;
          double process_sum = 0.75;
          double value = *tmp_value + process_sum;
          scalars->SetTuple1(index, value);//update intensity values

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
    //test vectors view transform
    Eigen::Vector4d a(1,0,0,0);
    Eigen::Vector4d b(0,1,0,0);
    Eigen::Vector4d c(0,0,1,0);
    Eigen::Vector4d d(0,0,0,1);

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

    //print out transforms
    Eigen::Vector4d a1(0,0,0,0);
    Eigen::Vector4d b1(0,0,0,0);
    Eigen::Vector4d c1(0,0,0,0);
    Eigen::Vector4d d1(0,0,0,0);

    a1 = epose*a;
    b1 = epose*b;
    c1 = epose*c;
    d1 = epose*d;
    ROS_INFO("w0: %f w1: %f w2: %f w3: %f\n",a1[0], a1[1], a1[2],a1[3]);
    ROS_INFO("u0: %f u1: %f u2: %f u3: %f\n",b1[0], b1[1], b1[2],b1[3]);
    ROS_INFO("-v0: %f -v1:  %f -v2: %f -v3: %f\n",c1[0], c1[1], c1[2],c1[3]);
    ROS_INFO("pt x:%f y:%f Z:%f last:%f\n",d1[0], d1[1], d1[2],d1[3]);
    ROS_INFO("");

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
