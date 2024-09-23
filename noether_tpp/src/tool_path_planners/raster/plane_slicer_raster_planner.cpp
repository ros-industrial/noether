#include <noether_tpp/tool_path_planners/raster/plane_slicer_raster_planner.h>
#include <noether_tpp/utils.h>

#include <algorithm>  // std::find(), std::unique()
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
#include <vtkErrorCode.h>
#include <vtkKdTreePointLocator.h>
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
    cutter->SetInputData(mesh_data);
    cutter->SetSortBy(1);
    cutter->SetGenerateTriangles(false);
    cutter->Update();

    stripper->SetInputConnection(cutter->GetOutputPort());
    stripper->JoinContiguousSegmentsOn();
    stripper->SetMaximumLength(mesh_data->GetNumberOfPoints());
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
  kd_tree_->SetDataSet(mesh_data);
  kd_tree_->BuildLocator();
  vtkSmartPointer<vtkCellLocator> cell_locator_ = vtkSmartPointer<vtkCellLocator>::New();
  cell_locator_->SetDataSet(mesh_data);
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
#if VTK_MAJOR_VERSION > 7
    const vtkIdType* indices;
#else
    vtkIdType* indices;
#endif
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
      double line_length = ::computeLength(points);
      if (line_length > min_segment_size_ && points->GetNumberOfPoints() > 1)
      {
        // enforce point spacing
        vtkSmartPointer<vtkPoints> new_points = applyParametricSpline(points, line_length, point_spacing_);

        // add points to segment now
        vtkSmartPointer<vtkPolyData> segment_data = vtkSmartPointer<vtkPolyData>::New();
        segment_data->SetPoints(new_points);

        // inserting normals
        if (!insertNormals(search_radius_, mesh_data, kd_tree_, segment_data))
        {
          throw std::runtime_error("Could not insert normals for segment " + std::to_string(r.raster_segments.size()) +
                                   " of raster " + std::to_string(i));
        }

        // saving into raster
        r.raster_segments.push_back(segment_data);
        r.segment_lengths.push_back(line_length);
      }
    }

    // Save raster
    if (!r.raster_segments.empty())
      rasters_data_vec.push_back(r);
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
