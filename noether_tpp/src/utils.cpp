#include <noether_tpp/utils.h>

#include <vtkCellData.h>
#include <vtkDoubleArray.h>
#include <vtkPointData.h>
#include <vtkPoints.h>
#include <vtkPolyDataNormals.h>

namespace noether
{
Eigen::Vector3d estimateToolPathDirection(const ToolPath& tool_path)
{
  Eigen::Vector3d avg_dir = Eigen::Vector3d::Zero();
  int n = 0;
  for (const ToolPathSegment& seg : tool_path)
  {
    if (seg.size() > 1)
    {
      for (std::size_t i = 0; i < seg.size() - 1; ++i)
      {
        avg_dir += (seg.at(i + 1).translation() - seg.at(i).translation()).normalized();
        ++n;
      }
    }
  }

  if (n < 1)
    throw std::runtime_error("Insufficient number of points to calculate average tool path direction");

  avg_dir /= static_cast<double>(n);

  return avg_dir.normalized();
}

Eigen::Vector3d estimateRasterDirection(const ToolPaths& tool_paths, const Eigen::Vector3d& reference_tool_path_dir)
{
  // First waypoint in the first segment of the first tool path
  const Eigen::Isometry3d first_wp = tool_paths.at(0).at(0).at(0);
  // First waypoint in the first segment of the last tool path
  const Eigen::Isometry3d first_wp_last_path = tool_paths.at(tool_paths.size() - 1).at(0).at(0);

  // Normalize the reference tool path direction
  const Eigen::Vector3d norm_ref_tool_path_dir = reference_tool_path_dir.normalized();

  // Get the vector between the two points
  Eigen::Vector3d raster_dir = first_wp_last_path.translation() - first_wp.translation();

  // Subtract the component of this vector that is parallel to the reference tool path direction
  raster_dir -= raster_dir.dot(norm_ref_tool_path_dir) * norm_ref_tool_path_dir;
  return raster_dir.normalized();
}

std::vector<pcl::PCLPointField>::const_iterator findField(const std::vector<pcl::PCLPointField>& fields,
                                                          const std::string& name)
{
  return std::find_if(
      fields.begin(), fields.end(), [&name](const pcl::PCLPointField& field) { return field.name == name; });
}

std::vector<pcl::PCLPointField>::const_iterator findFieldOrThrow(const std::vector<pcl::PCLPointField>& fields,
                                                                 const std::string& name)
{
  auto it = findField(fields, name);
  if (it == fields.end())
    throw std::runtime_error("Failed to find field '" + name + "'");
  return it;
}

bool hasNormals(const pcl::PolygonMesh& mesh)
{
  auto nx_it = noether::findField(mesh.cloud.fields, "normal_x");
  auto ny_it = noether::findField(mesh.cloud.fields, "normal_y");
  auto nz_it = noether::findField(mesh.cloud.fields, "normal_z");

  return nx_it != mesh.cloud.fields.end() && ny_it != mesh.cloud.fields.end() && nz_it != mesh.cloud.fields.end();
}

Eigen::Vector3f getPoint(const pcl::PCLPointCloud2& cloud, const std::uint32_t pt_idx)
{
  // Find the x, y, and z fields
  auto x_it = noether::findFieldOrThrow(cloud.fields, "x");
  auto y_it = noether::findFieldOrThrow(cloud.fields, "y");
  auto z_it = noether::findFieldOrThrow(cloud.fields, "z");

  // Check that the xyz fields are floats and contiguous
  if ((y_it->offset - x_it->offset != 4) || (z_it->offset - y_it->offset != 4))
    throw std::runtime_error("XYZ fields are not contiguous floats");

  const std::uint32_t offset = pt_idx * cloud.point_step;
  const auto* xyz = reinterpret_cast<const float*>(cloud.data.data() + offset + x_it->offset);
  return Eigen::Map<const Eigen::Vector3f>(xyz);
}

Eigen::Vector3f getNormal(const pcl::PCLPointCloud2& cloud, const std::uint32_t pt_idx)
{
  auto nx_it = noether::findField(cloud.fields, "normal_x");
  auto ny_it = noether::findField(cloud.fields, "normal_y");
  auto nz_it = noether::findField(cloud.fields, "normal_z");

  if (nx_it == cloud.fields.end() || ny_it == cloud.fields.end() || nz_it == cloud.fields.end())
    throw std::runtime_error("Not all normals exist");

  const std::uint32_t offset = pt_idx * cloud.point_step;
  const auto* nx = reinterpret_cast<const float*>(cloud.data.data() + offset + nx_it->offset);
  const auto* ny = reinterpret_cast<const float*>(cloud.data.data() + offset + ny_it->offset);
  const auto* nz = reinterpret_cast<const float*>(cloud.data.data() + offset + nz_it->offset);

  return Eigen::Vector3f(*nx, *ny, *nz);
}

TriangleMesh createTriangleMesh(const pcl::PolygonMesh& input)
{
  TriangleMesh mesh;

  // Add the vertices
  for (std::uint32_t i = 0; i < input.cloud.width * input.cloud.height; ++i)
  {
    mesh.addVertex(i);
  }

  // Add the faces
  for (auto poly = input.polygons.begin(); poly != input.polygons.end(); ++poly)
  {
    if (poly->vertices.size() < 3)
      throw std::runtime_error("Polygon has fewer than 3 vertices");

    // Create individual triangles from the polygon (n_vert - 2 total triangles)
    for (std::size_t tri = 0; tri < poly->vertices.size() - 2; ++tri)
    {
      TriangleMesh::VertexIndices triangle;
      triangle.resize(3);

      // Get the vertices of a triangle with the first point as the common point
      for (std::uint32_t i = 0; i < 3; ++i)
      {
        std::uint32_t v_ind;
        switch (i)
        {
          case 0:
            v_ind = poly->vertices[0];
            break;
          case 1:
            v_ind = poly->vertices[tri + 1];
            break;
          case 2:
            v_ind = poly->vertices[tri + 2];
            break;
        }

        triangle[i] = TriangleMesh::VertexIndex(v_ind);
      }

      mesh.addFace(triangle);
    }
  }

  return mesh;
}

std::tuple<double, std::vector<double>> computeLength(const ToolPathSegment& segment)
{
  double length = 0.0;

  // Set up a container for the distance along the segment of each waypoint
  std::vector<double> dists;
  dists.reserve(segment.size());

  // Add the distance of the first waypoint
  dists.push_back(0.0);

  for (std::size_t i = 0; i < segment.size() - 1; ++i)
  {
    const Eigen::Isometry3d& first = segment.at(i);
    const Eigen::Isometry3d& second = segment.at(i + 1);
    double d = (second.translation() - first.translation()).norm();
    length += d;
    dists.push_back(length);
  }

  return std::make_tuple(length, dists);
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

}  // namespace noether
