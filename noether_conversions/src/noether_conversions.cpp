#include <noether_conversions/noether_conversions.h>
#include <Eigen/Geometry>
#include <vtkPointData.h>
#include <pcl/conversions.h>
#include <pcl/point_types.h>
#include <pcl/io/ply_io.h>
#include <eigen_conversions/eigen_msg.h>
#include <console_bridge/console.h>

namespace noether_conversions
{

bool convertToPCLMesh(const shape_msgs::Mesh& mesh_msg, pcl::PolygonMesh& mesh)
{
  // iterating over triangles
  pcl::PointCloud<pcl::PointXYZ> mesh_points;
  mesh.polygons.clear();
  for(auto& triangle : mesh_msg.triangles)
  {
    pcl::Vertices vertices;
    vertices.vertices.assign(triangle.vertex_indices.begin(),triangle.vertex_indices.end());
    mesh.polygons.push_back(vertices);
  }

  std::transform(mesh_msg.vertices.begin(),mesh_msg.vertices.end(),std::back_inserter(mesh_points.points),[](
      const geometry_msgs::Point& point){
    pcl::PointXYZ p;
    std::tie(p.x,p.y,p.z) = std::make_tuple(point.x,point.y,point.z);
    return p;
  });

  pcl::toPCLPointCloud2(mesh_points,mesh.cloud);
  return true;
}

bool convertToMeshMsg(const pcl::PolygonMesh& mesh, shape_msgs::Mesh& mesh_msg)
{
  if(mesh.polygons.empty())
  {
    CONSOLE_BRIDGE_logError("PolygonMesh has no polygons");
    return false;
  }


  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromPCLPointCloud2(mesh.cloud,cloud);
  if(cloud.empty())
  {
    CONSOLE_BRIDGE_logError("PolygonMesh has no vertices data");
    return false;
  }

  // copying triangles
  mesh_msg.triangles.resize(mesh.polygons.size());
  for(std::size_t i = 0; i < mesh.polygons.size(); i++)
  {
    const pcl::Vertices& vertices = mesh.polygons[i];
    if(vertices.vertices.size() != 3)
    {
      CONSOLE_BRIDGE_logError("Vertex in PolygonMesh needs to have 3 elements only");
      return false;
    }

    boost::array<uint32_t, 3>& vertex = mesh_msg.triangles[i].vertex_indices;
    std::tie(vertex[0], vertex[1], vertex[2]) = std::make_tuple(vertices.vertices[0],
                                                                vertices.vertices[1],
                                                                vertices.vertices[2]);
  }

  // copying vertices
  mesh_msg.vertices.resize(cloud.size());
  std::transform(cloud.begin(),cloud.end(),mesh_msg.vertices.begin(),[](pcl::PointXYZ& v){
    geometry_msgs::Point p;
    std::tie(p.x, p.y, p.z) = std::make_tuple(v.x,v.y,v.z);
    return std::move(p);
  });
  return true;
}

bool savePLYFile(const std::string& filename, const shape_msgs::Mesh& mesh_msg)
{
  pcl::PolygonMesh mesh;
  return convertToPCLMesh(mesh_msg, mesh) && pcl::io::savePLYFile(filename,mesh) > 0;
}

bool loadPLYFile(const std::string& filename, shape_msgs::Mesh& mesh_msg)
{
  pcl::PolygonMesh mesh;
  return (pcl::io::loadPLYFile(filename,mesh) > 0) && convertToMeshMsg(mesh,mesh_msg);
}

}
