#include <noether_tpp/tool_path_planners/raster/plane_slicer_raster_planner.h>
#include <noether_tpp/utils.h>

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
#include <vtkCutter.h>
#include <vtkErrorCode.h>
#include <vtkPlane.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyData.h>
#include <vtkPolyDataNormals.h>
#include <vtkSmartPointer.h>
#include <vtkStripper.h>
#ifndef VTK_MAJOR_VERSION
#include <vtkVersionMacros.h>
#endif

namespace
{
static const double EPSILON = 1e-6;

Eigen::Matrix3d computeRotation(const Eigen::Vector3d& vx, const Eigen::Vector3d& vy, const Eigen::Vector3d& vz)
{
  Eigen::Matrix3d m;
  m.setIdentity();
  m.col(0) = vx.normalized();
  m.col(1) = vy.normalized();
  m.col(2) = vz.normalized();
  return m;
}

noether::ToolPaths convertToPoses(const std::vector<std::vector<vtkSmartPointer<vtkPolyData>>>& rasters_data)
{
  noether::ToolPaths rasters_array;
  for (const std::vector<vtkSmartPointer<vtkPolyData>>& rd : rasters_data)
  {
    noether::ToolPath raster_path;
    std::vector<vtkSmartPointer<vtkPolyData>> raster_segments;
    raster_segments.assign(rd.begin(), rd.end());

    for (const vtkSmartPointer<vtkPolyData>& polydata : raster_segments)
    {
      noether::ToolPathSegment raster_path_segment;
      std::size_t num_points = polydata->GetNumberOfPoints();
      Eigen::Vector3d p, p_next, vx, vy, vz;
      Eigen::Isometry3d pose;
      std::vector<int> indices(num_points);
      std::iota(indices.begin(), indices.end(), 0);
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

/**
 * @brief Gets the distances (normal to the cut plane) from the cut origin to the closest and furthest corners of the
 * mesh
 * @param obb_size Column-wise matrix of the object-aligned size vectors of the mesh (from PCA), relative to the mesh
 * origin
 * @param pca_centroid Centroid of the mesh determined by PCA, relative to the mesh origin
 * @param origin Origin of the raster pattern, relative to the mesh origin
 * @param dir Raster cut plane normal, relative to the mesh origin
 * @return Tuple of (min distance, max distance)
 */
static std::tuple<double, double> getDistancesToMinMaxCuts(const Eigen::Matrix3d& obb_size,
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

}  // namespace

namespace noether
{
PlaneSlicerRasterPlanner::PlaneSlicerRasterPlanner(DirectionGenerator::ConstPtr dir_gen,
                                                   OriginGenerator::ConstPtr origin_gen)
  : RasterPlanner(std::move(dir_gen), std::move(origin_gen))
{
}

void PlaneSlicerRasterPlanner::setMinSegmentSize(const double min_segment_size)
{
  min_segment_size_ = min_segment_size;
}

void PlaneSlicerRasterPlanner::setSearchRadius(const double search_radius) { search_radius_ = search_radius; }

void PlaneSlicerRasterPlanner::generateRastersBidirectionally(const bool bidirectional)
{
  bidirectional_ = bidirectional;
}

ToolPaths PlaneSlicerRasterPlanner::planImpl(const pcl::PolygonMesh& mesh) const
{
  if (!hasNormals(mesh))
    throw std::runtime_error("Mesh does not have vertex normals");

  // Convert input mesh to VTK type & calculate normals if necessary
  vtkSmartPointer<vtkPolyData> mesh_data_ = vtkSmartPointer<vtkPolyData>::New();
  pcl::VTKUtils::mesh2vtk(mesh, mesh_data_);
  mesh_data_->BuildLinks();
  mesh_data_->BuildCells();

  // Use principal component analysis (PCA) to determine the principal axes of the mesh
  Eigen::Vector3d mesh_normal;  // Unit vector along shortest mesh PCA direction
  Eigen::Matrix3d pca_vecs;     // Principal axes, scaled to the size of the mesh in each direction
  Eigen::Vector3d centroid;     // Mesh centroid
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

    mesh_normal = pca.getEigenVectors().col(2).cast<double>().normalized();
    centroid = pca.getMean().head<3>().cast<double>();
    pca_vecs = (pca.getEigenVectors().array().rowwise() * scales.transpose()).cast<double>();
  }

  // Get the initial cutting plane
  const Eigen::Vector3d cut_direction = dir_gen_->generate(mesh);
  const Eigen::Vector3d cut_normal = (cut_direction.normalized().cross(mesh_normal)).normalized();

  // Get the initial plane starting location
  const Eigen::Vector3d cut_origin = origin_gen_->generate(mesh);

  double d_min_cut, d_max_cut;
  std::tie(d_min_cut, d_max_cut) = getDistancesToMinMaxCuts(pca_vecs, centroid, cut_origin, cut_normal);

  // If we don't want to generate rasters bidirectionally...
  if (!bidirectional_)
  {
    // ... and the furthest cut distance is behind the cutting plane, then don't make any cuts
    if (d_max_cut < 0.0)
      return {};

    // ... and the closest cut distance is behined the cutting plane, set the distance to the closest cutting plane
    // equal to zero (i.e., at the nominal cut origin)
    if (d_min_cut < 0.0)
      d_min_cut = 0.0;
  }

  const double cut_span = std::abs(d_max_cut - d_min_cut);
  const auto num_planes = static_cast<std::size_t>(std::ceil(cut_span / line_spacing_));
  const Eigen::Vector3d start_loc = cut_origin + cut_normal * d_min_cut;

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
    cutter->SetSortByToSortByCell();
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

  // collect rasters and set direction
  raster_data->Update();
  vtkIdType num_slices = raster_data->GetTotalNumberOfInputConnections();
  std::vector<std::vector<vtkSmartPointer<vtkPolyData>>> rasters_data_vec;
  for (std::size_t i = 0; i < num_slices; i++)
  {
    vtkSmartPointer<vtkPolyData> raster_lines = raster_data->GetInput(i);
    if (raster_lines->GetNumberOfLines() == 0)
      continue;

    // Save raster
    rasters_data_vec.push_back({raster_lines});
  }

  // converting to poses msg now
  ToolPaths tool_paths = convertToPoses(rasters_data_vec);
  return tool_paths;
}

ToolPathPlanner::ConstPtr PlaneSlicerRasterPlannerFactory::create() const
{
  auto planner = std::make_unique<PlaneSlicerRasterPlanner>(direction_gen(), origin_gen());
  planner->setLineSpacing(line_spacing);
  planner->setPointSpacing(point_spacing);
  planner->setMinHoleSize(min_hole_size);
  planner->setSearchRadius(search_radius);
  planner->setMinSegmentSize(min_segment_size);
  planner->generateRastersBidirectionally(bidirectional);

  return std::move(planner);
}

}  // namespace noether
