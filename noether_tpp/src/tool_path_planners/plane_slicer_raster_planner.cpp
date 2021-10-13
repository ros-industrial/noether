#include <noether_tpp/tool_path_planners/raster/plane_slicer_raster_planner.h>

#include <algorithm>  // std::find(), std::reverse(), std::unique()
#include <numeric>    // std::iota()
#include <stdexcept>  // std::runtime_error
#include <string>     // std::to_string()
#include <utility>    // std::move()
#include <vector>     // std::vector

#include <pcl/common/common.h>  // pcl::getMinMax3d()
#include <pcl/common/pca.h>     // pcl::PCA
#include <pcl/conversions.h>    // pcl::fromPCLPointCloud2
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <vtkAppendPolyData.h>
#include <vtkCellArray.h>
#include <vtkCellData.h>
#include <vtkCellLocator.h>
#include <vtkCutter.h>
#include <vtkDoubleArray.h>
#include <vtkErrorCode.h>
#include <vtkKdTreePointLocator.h>
#include <vtkParametricSpline.h>
#include <vtkPlane.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataNormals.h>
#include <vtkSmartPointer.h>
#include <vtkStripper.h>

#include <noether_tpp/tool_path_modifiers/organization_modifiers.h>
#include <noether_tpp/tool_path_modifiers/waypoint_orientation_modifiers.h>

namespace
{
static const double EPSILON = 1e-6;

struct RasterConstructData
{
  std::vector<vtkSmartPointer<vtkPolyData>> raster_segments;
  std::vector<double> segment_lengths;
};

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

/**
 * @brief removes points that appear in multiple lists such that only one instance of that point
 *        index remains
 * @param points_lists
 */
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
        vz = vx.cross(vy).normalized();
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
}

namespace noether
{
PlaneSlicerRasterPlanner::PlaneSlicerRasterPlanner(std::unique_ptr<DirectionGenerator> dir_gen, std::unique_ptr<OriginGenerator> origin_gen)
  : RasterPlanner(std::move(dir_gen), std::move(origin_gen))
{
}

void PlaneSlicerRasterPlanner::setMinSegmentSize(const double& min_segment_size) { min_segment_size_ = min_segment_size; }
void PlaneSlicerRasterPlanner::setSearchRadius(const double& search_radius) { search_radius_ = search_radius; }

ToolPaths PlaneSlicerRasterPlanner::planImpl(const pcl::PolygonMesh& mesh) const
{
  // Convert input mesh to VTK type & calculate normals if necessary
  vtkSmartPointer<vtkPolyData> mesh_data_ = vtkSmartPointer<vtkPolyData>::New();
  pcl::VTKUtils::mesh2vtk(mesh, mesh_data_);
  mesh_data_->BuildLinks();
  mesh_data_->BuildCells();
  if (!mesh_data_->GetPointData()->GetNormals() || !mesh_data_->GetCellData()->GetNormals())
  {
    vtkSmartPointer<vtkPolyDataNormals> normal_generator = vtkSmartPointer<vtkPolyDataNormals>::New();
    normal_generator->SetInputData(mesh_data_);
    normal_generator->ComputePointNormalsOn();
    normal_generator->SetComputeCellNormals(!mesh_data_->GetCellData()->GetNormals());
    normal_generator->SetFeatureAngle(M_PI_2);
    normal_generator->SetSplitting(true);
    normal_generator->SetConsistency(true);
    normal_generator->SetAutoOrientNormals(false);
    normal_generator->SetFlipNormals(false);
    normal_generator->SetNonManifoldTraversal(false);
    normal_generator->Update();

    if (!mesh_data_->GetPointData()->GetNormals())
    {
      mesh_data_->GetPointData()->SetNormals(normal_generator->GetOutput()->GetPointData()->GetNormals());
    }

    if (!mesh_data_->GetCellData()->GetNormals())
    {
      mesh_data_->GetCellData()->SetNormals(normal_generator->GetOutput()->GetCellData()->GetNormals());
    }
  }

  // Use principal component analysis (PCA) to determine the principal axes of the mesh
  Eigen::Matrix3d pca_vecs;  // Principal axes, scaled to the size of the mesh in each direction
  Eigen::Vector3d centroid;  // Mesh centroid
  {
    // Use Original PCL point cloud
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::fromPCLPointCloud2(mesh.cloud, *cloud);

    // Perform PCA analysis
    pcl::PCA<pcl::PointXYZ> pca;
    pca.setInputCloud(cloud);

    // Get the extents of the point cloud relative to its mean and principal axes
    pcl::PointCloud<pcl::PointXYZ> proj;
    pca.project(*cloud, proj);
    pcl::PointXYZ min, max;
    pcl::getMinMax3D(proj, min, max);
    Eigen::Array3f scales = max.getArray3fMap() - min.getArray3fMap();

    centroid = pca.getMean().head<3>().cast<double>();
    pca_vecs = (pca.getEigenVectors().array().rowwise() * scales.transpose()).cast<double>();
  }

  // Get the initial cutting plane
  Eigen::Vector3d cut_direction = dir_gen_->generate(mesh);
  Eigen::Vector3d cut_normal = (cut_direction.normalized().cross(pca_vecs.col(2).normalized())).normalized();

  // Calculate the number of planes needed to cover the mesh according to the length of the principle axes
  double max_extent = std::sqrt(pca_vecs.col(0).squaredNorm() + pca_vecs.col(1).squaredNorm() + pca_vecs.col(2).squaredNorm());
  std::size_t num_planes = static_cast<std::size_t>(max_extent / line_spacing_);

  // Calculate the start location as the centroid less half the full diagonal of the bounding box
  Eigen::Vector3d start_loc = centroid - (max_extent / 2.0) * cut_normal;

  // Generate each plane and collect the intersection points
  vtkSmartPointer<vtkAppendPolyData> raster_data = vtkSmartPointer<vtkAppendPolyData>::New();
  for (std::size_t i = 0; i < num_planes + 1; i++)
  {
    vtkSmartPointer<vtkPlane> plane = vtkSmartPointer<vtkPlane>::New();
    vtkSmartPointer<vtkCutter> cutter = vtkSmartPointer<vtkCutter>::New();
    vtkSmartPointer<vtkStripper> stripper = vtkSmartPointer<vtkStripper>::New();

    Eigen::Vector3d current_loc = start_loc + i * line_spacing_ * cut_normal;
    plane->SetOrigin(current_loc.x(), current_loc.y(), current_loc.z());
    plane->SetNormal(cut_normal.x(), cut_normal.y(), cut_normal.z());

    cutter->SetCutFunction(plane);
    cutter->SetInputData(mesh_data_);
    cutter->SetSortBy(1);
    cutter->SetGenerateTriangles(false);
    cutter->Update();

    stripper->SetInputConnection(cutter->GetOutputPort());
    stripper->JoinContiguousSegmentsOn();
    stripper->SetMaximumLength(mesh_data_->GetNumberOfPoints());
    stripper->Update();

    if (stripper->GetErrorCode() != vtkErrorCode::NoError)
    {
      continue;
    }

    for (int r = 0; r < stripper->GetNumberOfOutputPorts(); r++)
    {
      raster_data->AddInputData(stripper->GetOutput(r));
    }
  }

  // build cell locator and kd_tree to recover normals later on
  vtkSmartPointer<vtkKdTreePointLocator> kd_tree_ = vtkSmartPointer<vtkKdTreePointLocator>::New();
  kd_tree_->SetDataSet(mesh_data_);
  kd_tree_->BuildLocator();
  vtkSmartPointer<vtkCellLocator> cell_locator_ = vtkSmartPointer<vtkCellLocator>::New();
  cell_locator_->SetDataSet(mesh_data_);
  cell_locator_->BuildLocator();

  // collect rasters and set direction
  raster_data->Update();
  vtkIdType num_slices = raster_data->GetTotalNumberOfInputConnections();
  std::vector<RasterConstructData> rasters_data_vec;
  std::vector<std::vector<vtkIdType>> raster_ids;
  for (std::size_t i = 0; i < num_slices; i++)
  {
    RasterConstructData r;

    // collecting raster segments based on min hole size
    vtkSmartPointer<vtkPolyData> raster_lines = raster_data->GetInput(i);
    vtkIdType* indices;
    vtkIdType num_points;
    vtkIdType num_lines = raster_lines->GetNumberOfLines();
    vtkCellArray* cells = raster_lines->GetLines();

    if (num_lines == 0)
    {
      continue;
    }

    raster_ids.clear();

    unsigned int lineCount = 0;
    for (cells->InitTraversal(); cells->GetNextCell(num_points, indices); lineCount++)
    {
      std::vector<vtkIdType> point_ids;
      for (vtkIdType i = 0; i < num_points; i++)
      {
        if (std::find(point_ids.begin(), point_ids.end(), indices[i]) != point_ids.end())
        {
          continue;
        }
        point_ids.push_back(indices[i]);
      }
      if (point_ids.empty())
      {
        continue;
      }

      // removing duplicates
      auto iter = std::unique(point_ids.begin(), point_ids.end());
      point_ids.erase(iter, point_ids.end());

      // adding
      raster_ids.push_back(point_ids);
    }

    if (raster_ids.empty())
    {
      continue;
    }

    // remove redundant indices
    removeRedundant(raster_ids);

    // merging segments
    mergeRasterSegments(raster_lines->GetPoints(), min_hole_size_, raster_ids);

    // rectifying
    if (rasters_data_vec.size() > 1)
    {
      Eigen::Vector3d ref_point;
      rasters_data_vec.back().raster_segments.front()->GetPoint(0, ref_point.data());  // first point in previous raster
      rectifyDirection(raster_lines->GetPoints(), ref_point, raster_ids);
    }

    for (auto& rpoint_ids : raster_ids)
    {
      // Populating with points
      vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
      std::for_each(rpoint_ids.begin(), rpoint_ids.end(), [&points, &raster_lines](vtkIdType& id) {
        std::array<double, 3> p;
        raster_lines->GetPoint(id, p.data());
        points->InsertNextPoint(p.data());
      });

      // compute length and add points if segment length is greater than threshold
      double line_length = computeLength(points);
      if (line_length > min_segment_size_ && points->GetNumberOfPoints() > 1)
      {
        // enforce point spacing
        vtkSmartPointer<vtkPoints> new_points = applyParametricSpline(points, line_length, point_spacing_);

        // add points to segment now
        vtkSmartPointer<vtkPolyData> segment_data = vtkSmartPointer<vtkPolyData>::New();
        segment_data->SetPoints(new_points);

        // inserting normals
        if (!insertNormals(search_radius_, mesh_data_, kd_tree_, segment_data))
        {
          throw std::runtime_error("Could not insert normals for segment " + std::to_string(r.raster_segments.size()) + " of raster " + std::to_string(i));
        }

        // saving into raster
        r.raster_segments.push_back(segment_data);
        r.segment_lengths.push_back(line_length);
      }
    }

    // Save raster
    rasters_data_vec.push_back(r);
  }

  // converting to poses msg now
  ToolPaths tool_paths = convertToPoses(rasters_data_vec);
  return tool_paths;
}

std::unique_ptr<const ToolPathPlanner> PlaneSlicerRasterPlannerFactory::create() const
{
  std::unique_ptr<PlaneSlicerRasterPlanner> planner =
      std::make_unique<PlaneSlicerRasterPlanner>(dir_gen->clone(),
                                                 origin_gen->clone());
  planner->setPointSpacing(point_spacing);
  planner->setLineSpacing(line_spacing);
  planner->setMinHoleSize(min_hole_size);
  planner->setMinSegmentSize(min_segment_size);
  planner->setSearchRadius(search_radius);
  return planner;
}

}  // namespace noether
