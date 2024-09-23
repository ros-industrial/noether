#include <noether_tpp/tool_path_planners/raster/raster_utils.h>

#include <algorithm>  // std::reverse(), std::unique()
#include <numeric>    // std::iota()
#include <vector>     // std::vector

#include <pcl/common/pca.h>
#include <pcl/conversions.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>

#include <vtkCellData.h>
#include <vtkDoubleArray.h>
#include <vtkKdTreePointLocator.h>
#include <vtkParametricSpline.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataNormals.h>
#include <vtkSmartPointer.h>


namespace noether
{

Eigen::Matrix3d computeRotation(const Eigen::Vector3d& vx, const Eigen::Vector3d& vy, const Eigen::Vector3d& vz)
{
  Eigen::Matrix3d m;
  m.setIdentity();
  m.col(0) = vx.normalized();
  m.col(1) = vy.normalized();
  m.col(2) = vz.normalized();
  return m;
}

double computeLength(const vtkSmartPointer<vtkPoints>& points)
{
  const vtkIdType num_points = points->GetNumberOfPoints();
  double total_length = 0.0;
  if (num_points < 2)
  {
    return total_length;
  }

  Eigen::Vector3d p0, pf;
  for (vtkIdType i = 1; i < num_points; i++)
  {
    points->GetPoint(i - 1, p0.data());
    points->GetPoint(i, pf.data());

    total_length += (pf - p0).norm();
  }
  return total_length;
}

vtkSmartPointer<vtkPoints> applyParametricSpline(const vtkSmartPointer<vtkPoints>& points,
                                                 double total_length,
                                                 double point_spacing)
{
  vtkSmartPointer<vtkPoints> new_points = vtkSmartPointer<vtkPoints>::New();

  // create spline
  vtkSmartPointer<vtkParametricSpline> spline = vtkSmartPointer<vtkParametricSpline>::New();
  spline->SetPoints(points);
  spline->SetParameterizeByLength(true);
  spline->ClosedOff();

  // adding first point
  Eigen::Vector3d pt_prev;
  points->GetPoint(0, pt_prev.data());
  new_points->InsertNextPoint(pt_prev.data());

  // adding remaining points by evaluating spline
  std::size_t num_points = static_cast<std::size_t>(std::ceil(total_length / point_spacing) + 1);
  double du[9];
  Eigen::Vector3d u, pt;
  for (unsigned i = 1; i < num_points; i++)
  {
    double interv = static_cast<double>(i) / static_cast<double>(num_points - 1);
    interv = interv > 1.0 ? 1.0 : interv;
    if (std::abs(interv - 1.0) < EPSILON)
    {
      break;  // reach end
    }

    u = interv * Eigen::Vector3d::Ones();
    std::tie(u[0], u[1], u[2]) = std::make_tuple(interv, interv, interv);
    spline->Evaluate(u.data(), pt.data(), du);

    // check distance
    if ((pt - pt_prev).norm() >= point_spacing)
    {
      new_points->InsertNextPoint(pt.data());
      pt_prev = pt;
    }
  }

  // add last point
  points->GetPoint(points->GetNumberOfPoints() - 1, pt_prev.data());
  new_points->InsertNextPoint(pt_prev.data());

  return new_points;
}

void removeRedundant(std::vector<std::vector<vtkIdType>>& points_lists)
{
  using IdList = std::vector<vtkIdType>;
  if (points_lists.size() < 2)
  {
    return;
  }

  std::vector<std::vector<vtkIdType>> new_points_lists;
  new_points_lists.push_back(points_lists.front());
  for (std::size_t i = 1; i < points_lists.size(); i++)
  {
    IdList& current_list = points_lists[i];
    IdList new_list;
    IdList all_ids;

    // create list of all ids
    for (auto& ref_list : new_points_lists)
    {
      all_ids.insert(all_ids.end(), ref_list.begin(), ref_list.end());
    }

    for (auto& id : current_list)
    {
      // add if not found in any of the previous lists
      if (std::find(all_ids.begin(), all_ids.end(), id) == all_ids.end())
      {
        new_list.push_back(id);
      }
    }

    // add if it has enough points
    if (new_list.size() > 0)
    {
      new_points_lists.push_back(new_list);
    }
  }

  points_lists.clear();
  points_lists.assign(new_points_lists.begin(), new_points_lists.end());
}

void mergeRasterSegments(const vtkSmartPointer<vtkPoints>& points,
                         double merge_dist,
                         std::vector<std::vector<vtkIdType>>& points_lists)
{
  using namespace Eigen;
  using IdList = std::vector<vtkIdType>;
  if (points_lists.size() < 2)
  {
    return;
  }

  std::vector<IdList> new_points_lists;
  IdList merged_list_ids;
  IdList merged_list;

  auto do_merge =
      [&points](const IdList& current_list, const IdList& next_list, double merge_dist, IdList& merged_list) {
        Vector3d cl_point, nl_point;

        // checking front and back end points respectively
        points->GetPoint(current_list.front(), cl_point.data());
        points->GetPoint(next_list.back(), nl_point.data());
        double d = (cl_point - nl_point).norm();
        if (d < merge_dist)
        {
          merged_list.assign(next_list.begin(), next_list.end());
          merged_list.insert(merged_list.end(), current_list.begin(), current_list.end());
          return true;
        }

        // checking back and front end points respectively
        points->GetPoint(current_list.back(), cl_point.data());
        points->GetPoint(next_list.front(), nl_point.data());
        d = (cl_point - nl_point).norm();
        if (d < merge_dist)
        {
          merged_list.assign(current_list.begin(), current_list.end());
          merged_list.insert(merged_list.end(), next_list.begin(), next_list.end());
          return true;
        }
        return false;
      };

  for (std::size_t i = 0; i < points_lists.size(); i++)
  {
    if (std::find(merged_list_ids.begin(), merged_list_ids.end(), i) != merged_list_ids.end())
    {
      // already merged
      continue;
    }

    IdList current_list = points_lists[i];
    Vector3d cl_point, nl_point;
    bool seek_adjacent = true;
    while (seek_adjacent)
    {
      seek_adjacent = false;
      for (std::size_t j = i + 1; j < points_lists.size(); j++)
      {
        if (std::find(merged_list_ids.begin(), merged_list_ids.end(), j) != merged_list_ids.end())
        {
          // already merged
          continue;
        }

        merged_list.clear();
        IdList next_list = points_lists[j];
        if (do_merge(current_list, next_list, merge_dist, merged_list))
        {
          current_list = merged_list;
          merged_list_ids.push_back(static_cast<vtkIdType>(j));
          seek_adjacent = true;
          continue;
        }

        std::reverse(next_list.begin(), next_list.end());
        if (do_merge(current_list, next_list, merge_dist, merged_list))
        {
          current_list = merged_list;
          merged_list_ids.push_back(static_cast<vtkIdType>(j));
          seek_adjacent = true;
          continue;
        }
      }
    }
    new_points_lists.push_back(current_list);
  }
  points_lists.clear();
  std::copy_if(new_points_lists.begin(), new_points_lists.end(), std::back_inserter(points_lists), [](const IdList& l) {
    return l.size() > 1;
  });
}

void rectifyDirection(const vtkSmartPointer<vtkPoints>& points,
                      const Eigen::Vector3d& ref_point,
                      std::vector<std::vector<vtkIdType>>& points_lists)
{
  Eigen::Vector3d p0, pf;
  if (points_lists.size() < 2)
    return;

  // getting first and last points
  points->GetPoint(points_lists.front().front(), p0.data());
  points->GetPoint(points_lists.back().back(), pf.data());

  bool reverse = (ref_point - p0).norm() > (ref_point - pf).norm();
  if (reverse)
  {
    for (auto& s : points_lists)
    {
      std::reverse(s.begin(), s.end());
    }
    std::reverse(points_lists.begin(), points_lists.end());
  }
}

noether::ToolPaths convertToPoses(const std::vector<RasterConstructData>& rasters_data)
{
  noether::ToolPaths rasters_array;
  bool reverse = true;
  for (const RasterConstructData& rd : rasters_data)
  {
    reverse = !reverse;
    noether::ToolPath raster_path;
    std::vector<vtkSmartPointer<vtkPolyData>> raster_segments;
    raster_segments.assign(rd.raster_segments.begin(), rd.raster_segments.end());
    if (reverse)
    {
      std::reverse(raster_segments.begin(), raster_segments.end());
    }

    for (const vtkSmartPointer<vtkPolyData>& polydata : raster_segments)
    {
      noether::ToolPathSegment raster_path_segment;
      std::size_t num_points = polydata->GetNumberOfPoints();
      Eigen::Vector3d p, p_next, vx, vy, vz;
      Eigen::Isometry3d pose;
      std::vector<int> indices(num_points);
      std::iota(indices.begin(), indices.end(), 0);
      if (reverse)
      {
        std::reverse(indices.begin(), indices.end());
      }
      for (std::size_t i = 0; i < indices.size() - 1; i++)
      {
        int idx = indices[i];
        int idx_next = indices[i + 1];
        polydata->GetPoint(idx, p.data());
        polydata->GetPoint(idx_next, p_next.data());
        polydata->GetPointData()->GetNormals()->GetTuple(idx, vz.data());
        vx = (p_next - p).normalized();
        vy = vz.cross(vx).normalized();
        vx = vy.cross(vz).normalized();
        pose = Eigen::Translation3d(p) * Eigen::AngleAxisd(computeRotation(vx, vy, vz));
        raster_path_segment.push_back(pose);
      }

      // adding last pose
      pose.translation() = p_next;  // orientation stays the same as previous
      raster_path_segment.push_back(pose);

      raster_path.push_back(raster_path_segment);
    }

    if (!raster_path.empty())
      rasters_array.push_back(raster_path);
  }

  return rasters_array;
}

bool insertNormals(const double search_radius,
                   vtkSmartPointer<vtkPolyData>& mesh_data_,
                   vtkSmartPointer<vtkKdTreePointLocator>& kd_tree_,
                   vtkSmartPointer<vtkPolyData>& data)
{
  // Find closest cell to each point and uses its normal vector
  vtkSmartPointer<vtkDoubleArray> new_normals = vtkSmartPointer<vtkDoubleArray>::New();
  new_normals->SetNumberOfComponents(3);
  new_normals->SetNumberOfTuples(data->GetPoints()->GetNumberOfPoints());

  // get normal data
  vtkSmartPointer<vtkDataArray> normal_data = mesh_data_->GetPointData()->GetNormals();

  if (!normal_data)
  {
    return false;
  }

  Eigen::Vector3d normal_vect = Eigen::Vector3d::UnitZ();
  for (int i = 0; i < data->GetPoints()->GetNumberOfPoints(); ++i)
  {
    // locate closest cell
    Eigen::Vector3d query_point;
    vtkSmartPointer<vtkIdList> id_list = vtkSmartPointer<vtkIdList>::New();
    data->GetPoints()->GetPoint(i, query_point.data());
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
  data->GetPointData()->SetNormals(new_normals);
  return true;
}

std::tuple<double, double> getDistancesToMinMaxCuts(const Eigen::Matrix3d& obb_size,
                                                    const Eigen::Vector3d& obb_centroid,
                                                    const Eigen::Vector3d& cut_origin,
                                                    const Eigen::Vector3d& cut_normal)
{
  /* Create the 8 corners of the object-aligned bounding box using vectorization
   *
   * | x0  x1  x2 | * | 1 -1  1 -1  1 -1  1 -1 | +  | ox | = | cx0  cx1  ... |
   * | y0  y1  y2 |   | 1  1 -1 -1  1  1 -1 -1 |    | oy |   | cy0  cy1  ... |
   * | z0  z1  zz |   | 1  1  1  1 -1 -1 -1 -1 |    | oz |   | cz0  cz1  ... |
   */
  Eigen::MatrixXi corner_mask(3, 8);
  // clang-format off
  corner_mask <<
      1, -1, 1, -1, 1, -1, 1, -1,
      1, 1, -1, -1, 1, 1, -1, -1,
      1, 1, 1, 1, -1, -1, -1, -1;
  // clang-format on
  Eigen::MatrixXd corners = (obb_size / 2.0) * corner_mask.cast<double>();
  corners.colwise() += obb_centroid;

  // For each corner, compute the distance from the raster origin to a plane that passes through the OBB corner and
  // whose normal is the input raster direction. Save the largest and smallest distances to the corners
  double d_max = -std::numeric_limits<double>::max();
  double d_min = std::numeric_limits<double>::max();
  for (Eigen::Index col = 0; col < corners.cols(); ++col)
  {
    Eigen::Hyperplane<double, 3> plane(cut_normal, corners.col(col));
    double d = plane.signedDistance(cut_origin);

    // Negate the signed distance because points "in front" of the cut plane have to travel in the negative plane
    // direction to get back to the plane
    d *= -1;

    if (d > d_max)
      d_max = d;
    else if (d < d_min)
      d_min = d;
  }

  return std::make_tuple(d_min, d_max);
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

} // namespace noether
