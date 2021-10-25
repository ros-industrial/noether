#include <noether_filtering/utils.h>

#include <pcl/conversions.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/vtk_smoothing/vtk_utils.h>
#include <vtkPointData.h>

namespace noether_filtering
{
namespace utils
{

int vtk2TriangleMesh(const vtkSmartPointer<vtkPolyData>& poly_data, pcl::PolygonMesh& mesh)
{
  mesh.polygons.clear ();
  mesh.cloud.data.clear ();
  mesh.cloud.width = mesh.cloud.height = 0;
  mesh.cloud.is_dense = true;

  vtkSmartPointer<vtkPoints> mesh_points = poly_data->GetPoints ();
  vtkIdType nr_points = mesh_points->GetNumberOfPoints ();
  vtkIdType nr_polygons = poly_data->GetNumberOfPolys ();

  if (nr_points == 0)
    return 0;

  vtkUnsignedCharArray* poly_colors = nullptr;
  if (poly_data->GetPointData() != nullptr)
    poly_colors = vtkUnsignedCharArray::SafeDownCast (poly_data->GetPointData ()->GetScalars ("Colors"));

  // Some applications do not save the name of scalars (including PCL's native vtk_io)
  if (!poly_colors)
    poly_colors = vtkUnsignedCharArray::SafeDownCast (poly_data->GetPointData ()->GetScalars ("scalars"));

  // TODO: currently only handles rgb values with 3 components
  if (poly_colors && (poly_colors->GetNumberOfComponents () == 3))
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZRGB>());
    cloud_temp->points.resize (nr_points);
    double point_xyz[3];
    unsigned char point_color[3];
    for (vtkIdType i = 0; i < mesh_points->GetNumberOfPoints (); ++i)
    {
      mesh_points->GetPoint (i, &point_xyz[0]);
      (*cloud_temp)[i].x = static_cast<float> (point_xyz[0]);
      (*cloud_temp)[i].y = static_cast<float> (point_xyz[1]);
      (*cloud_temp)[i].z = static_cast<float> (point_xyz[2]);

      poly_colors->GetTupleValue (i, &point_color[0]);
      (*cloud_temp)[i].r = point_color[0];
      (*cloud_temp)[i].g = point_color[1];
      (*cloud_temp)[i].b = point_color[2];
    }
    cloud_temp->width = cloud_temp->size ();
    cloud_temp->height = 1;
    cloud_temp->is_dense = true;

    pcl::toPCLPointCloud2 (*cloud_temp, mesh.cloud);
  }
  else // in case points do not have color information:
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZ> ());
    cloud_temp->points.resize (nr_points);
    double point_xyz[3];
    for (vtkIdType i = 0; i < mesh_points->GetNumberOfPoints (); ++i)
    {
      mesh_points->GetPoint (i, &point_xyz[0]);
      (*cloud_temp)[i].x = static_cast<float> (point_xyz[0]);
      (*cloud_temp)[i].y = static_cast<float> (point_xyz[1]);
      (*cloud_temp)[i].z = static_cast<float> (point_xyz[2]);
    }
    cloud_temp->width = cloud_temp->size ();
    cloud_temp->height = 1;
    cloud_temp->is_dense = true;

    pcl::toPCLPointCloud2 (*cloud_temp, mesh.cloud);
  }

  mesh.polygons.reserve (nr_polygons);
#ifdef VTK_CELL_ARRAY_V2
  vtkIdType const *cell_points;
#else
  vtkIdType* cell_points;
#endif
  vtkIdType nr_cell_points;
  vtkCellArray * mesh_polygons = poly_data->GetPolys ();
  mesh_polygons->InitTraversal ();
  int id_poly = 0;
  while (mesh_polygons->GetNextCell (nr_cell_points, cell_points))
  {
    if (nr_cell_points == 3 &&
        cell_points[0] != cell_points[1] &&
        cell_points[1] != cell_points[2] &&
        cell_points[2] != cell_points[0])
    {
      mesh.polygons.emplace_back(pcl::Vertices());
      mesh.polygons[id_poly].vertices.resize (nr_cell_points);
      for (vtkIdType i = 0; i < nr_cell_points; ++i)
        mesh.polygons[id_poly].vertices[i] = static_cast<unsigned int> (cell_points[i]);
      ++id_poly;
    }
  }

  return (static_cast<int> (nr_points));
}

}  // namespace utils
}  // namespace noether_filtering
