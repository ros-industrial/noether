#include <noether_tpp/tool_path_planners/raster/raster_utils.h>

#include <pcl/surface/vtk_smoothing/vtk_utils.h>

#include <vtkCellData.h>
#include <vtkDoubleArray.h>
#include <vtkKdTreePointLocator.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataNormals.h>
#include <vtkSmartPointer.h>

namespace noether
{
bool insertNormals(const double search_radius,
                   vtkSmartPointer<vtkPolyData>& mesh_data_,
                   vtkSmartPointer<vtkKdTreePointLocator>& kd_tree_,
                   vtkSmartPointer<vtkPolyData>& segment_data)
{
  // Find closest cell to each point and uses its normal vector
  vtkSmartPointer<vtkDoubleArray> new_normals = vtkSmartPointer<vtkDoubleArray>::New();
  new_normals->SetNumberOfComponents(3);
  new_normals->SetNumberOfTuples(segment_data->GetPoints()->GetNumberOfPoints());

  // get normal data
  vtkSmartPointer<vtkDataArray> normal_data = mesh_data_->GetPointData()->GetNormals();

  if (!normal_data)
  {
    return false;
  }

  Eigen::Vector3d normal_vect = Eigen::Vector3d::UnitZ();
  for (int i = 0; i < segment_data->GetPoints()->GetNumberOfPoints(); ++i)
  {
    // locate closest cell
    Eigen::Vector3d query_point;
    vtkSmartPointer<vtkIdList> id_list = vtkSmartPointer<vtkIdList>::New();
    segment_data->GetPoints()->GetPoint(i, query_point.data());
    kd_tree_->FindPointsWithinRadius(search_radius, query_point.data(), id_list);
    if (id_list->GetNumberOfIds() < 1)
    {
      kd_tree_->FindClosestNPoints(1, query_point.data(), id_list);

      if (id_list->GetNumberOfIds() < 1)
      {
        return false;
      }
    }

    // compute normal average
    normal_vect = Eigen::Vector3d::Zero();
    std::size_t num_normals = 0;
    for (auto p = 0; p < id_list->GetNumberOfIds(); p++)
    {
      Eigen::Vector3d temp_normal, query_point, closest_point;
      vtkIdType p_id = id_list->GetId(p);

      if (p_id < 0)
      {
        continue;
      }

      // get normal and add it to average
      normal_data->GetTuple(p_id, temp_normal.data());
      normal_vect += temp_normal.normalized();
      num_normals++;
    }

    normal_vect /= num_normals;
    normal_vect.normalize();

    // save normal
    new_normals->SetTuple3(i, normal_vect(0), normal_vect(1), normal_vect(2));
  }
  segment_data->GetPointData()->SetNormals(new_normals);
  return true;
}

vtkSmartPointer<vtkPolyData> convertMeshToVTKPolyData(const pcl::PolygonMesh& mesh)
{
  // Convert input mesh to VTK type & calculate normals if necessary
  vtkSmartPointer<vtkPolyData> mesh_data = vtkSmartPointer<vtkPolyData>::New();
  pcl::VTKUtils::mesh2vtk(mesh, mesh_data);
  mesh_data->BuildLinks();
  mesh_data->BuildCells();

  if (!mesh_data->GetPointData()->GetNormals() || !mesh_data->GetCellData()->GetNormals())
  {
    vtkSmartPointer<vtkPolyDataNormals> normal_generator = vtkSmartPointer<vtkPolyDataNormals>::New();
    normal_generator->SetInputData(mesh_data);
    normal_generator->ComputePointNormalsOn();
    normal_generator->SetComputeCellNormals(!mesh_data->GetCellData()->GetNormals());
    normal_generator->SetFeatureAngle(M_PI_2);
    normal_generator->SetSplitting(true);
    normal_generator->SetConsistency(true);
    normal_generator->SetAutoOrientNormals(false);
    normal_generator->SetFlipNormals(false);
    normal_generator->SetNonManifoldTraversal(false);
    normal_generator->Update();

    if (!mesh_data->GetPointData()->GetNormals())
    {
      mesh_data->GetPointData()->SetNormals(normal_generator->GetOutput()->GetPointData()->GetNormals());
    }

    if (!mesh_data->GetCellData()->GetNormals())
    {
      mesh_data->GetCellData()->SetNormals(normal_generator->GetOutput()->GetCellData()->GetNormals());
    }
  }

  return mesh_data;
}

}  // namespace noether
