#include <noether_conversions/noether_conversions.h>
#include <type_traits>
#include <Eigen/Geometry>
#include <pcl/conversions.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <vtkPointData.h>

#include <console_bridge/console.h>
#include <eigen_conversions/eigen_msg.h>
#include <shape_msgs/MeshTriangle.h>

namespace noether_conversions
{

static geometry_msgs::Pose pose3DtoPoseMsg(const std::tuple<float,float,float,float,float,float>& p)
{
  using namespace Eigen;
  geometry_msgs::Pose pose_msg;
  Eigen::Affine3d eigen_pose = Translation3d(Vector3d(std::get<0>(p),std::get<1>(p),std::get<2>(p))) *
      AngleAxisd(std::get<3>(p),Vector3d::UnitX()) * AngleAxisd(std::get<4>(p),Vector3d::UnitY()) *
      AngleAxisd(std::get<5>(p),Vector3d::UnitZ());

  tf::poseEigenToMsg(eigen_pose,pose_msg);
  return std::move(pose_msg);
}

bool convertToPCLMesh(const shape_msgs::Mesh& mesh_msg, pcl::PolygonMesh& mesh)
{
  // iterating over triangles
  pcl::PointCloud<pcl::PointXYZ> mesh_points;
  mesh.polygons.clear();
  for (auto& triangle : mesh_msg.triangles)
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
  return convertToPCLMesh(mesh_msg, mesh) && pcl::io::savePLYFile(filename,mesh) >= 0;
}

bool loadPLYFile(const std::string& filename, shape_msgs::Mesh& mesh_msg)
{
  pcl::PolygonMesh mesh;
  return (pcl::io::loadPLYFile(filename,mesh) >= 0) && convertToMeshMsg(mesh,mesh_msg);
}

visualization_msgs::Marker createMeshMarker(const std::string& mesh_file, const std::string& ns,
                                            const std::string& frame_id,
                                            const std::tuple<double,double,double,double>& rgba)
{
  visualization_msgs::Marker m;
  m.action = m.ADD;
  m.id = 0;
  m.ns = ns;
  std::tie(m.color.r, m.color.g, m.color.b, m.color.a) = rgba;
  m.header.frame_id = frame_id;
  m.lifetime = ros::Duration(0);
  std::tie(m.scale.x , m.scale.y ,m.scale.z) = std::make_tuple(1.0,1.0,1.0);
  m.mesh_resource = "file://" + mesh_file;
  m.type = m.MESH_RESOURCE;
  return std::move(m);
}

void convertToPointNormals(const pcl::PolygonMesh& mesh, pcl::PointCloud<pcl::PointNormal>& cloud_normals, bool flip)
{
  using namespace pcl;
  using namespace Eigen;
  using PType = std::remove_reference<decltype(cloud_normals)>::type::PointType;
  PointCloud<PointXYZ> points;
  pcl::fromPCLPointCloud2<PointXYZ>(mesh.cloud, points);
  pcl::copyPointCloud(points, cloud_normals);

  // computing the normals by walking the vertices
  Vector3d a, b, c;
  Vector3d dir;
  for(std::size_t i = 0; i < mesh.polygons.size(); i++)
  {
    const std::vector<uint32_t>&  vert = mesh.polygons[i].vertices;
    a = points[vert[0]].getVector3fMap().cast<double>();
    b = points[vert[1]].getVector3fMap().cast<double>();
    c = points[vert[2]].getVector3fMap().cast<double>();
    dir = (b - a).cross((c -a)).normalized();
    dir = flip ? -1.0 * dir : dir;

    // assigning to points
    for(std::size_t v = 0; v < vert.size(); v++)
    {
      PointNormal& p = cloud_normals[vert[v]];
      p.normal_x = dir.x();
      p.normal_y = dir.y();
      p.normal_z = dir.z();
    }
  }
}

visualization_msgs::MarkerArray convertToAxisMarkers(const noether_msgs::ToolRasterPath& toolpath,
                                                     const std::string& frame_id, const std::string& ns,
                                                     const std::size_t start_id, const double axis_scale,
                                                     const double axis_length,
                                                     const std::tuple<float, float, float, float, float, float>& offset)
{
  return convertToAxisMarkers(toolpath.paths, frame_id, ns, start_id, axis_scale, axis_length, offset);
}

visualization_msgs::MarkerArray convertToAxisMarkers(const std::vector<geometry_msgs::PoseArray>& path,
                                                     const std::string& frame_id, const std::string& ns,
                                                     const std::size_t start_id, const double axis_scale,
                                                     const double axis_length,
                                                     const std::tuple<float, float, float, float, float, float>& offset)
{
  using namespace Eigen;

  visualization_msgs::MarkerArray markers;

  auto create_line_marker = [&](const int id, const std::tuple<float, float, float, float>& rgba) -> visualization_msgs::Marker
  {
    visualization_msgs::Marker line_marker;
    line_marker.action = line_marker.ADD;
    std::tie(line_marker.color.r, line_marker.color.g, line_marker.color.b, line_marker.color.a) = rgba;
    line_marker.header.frame_id = frame_id;
    line_marker.type = line_marker.LINE_LIST;
    line_marker.id = id;
    line_marker.lifetime = ros::Duration(0);
    line_marker.ns = ns;
    std::tie(line_marker.scale.x, line_marker.scale.y, line_marker.scale.z) = std::make_tuple(axis_scale, 0.0, 0.0);
    line_marker.pose = pose3DtoPoseMsg(offset);
    return std::move(line_marker);
  };

  // markers for each axis line
  int marker_id = start_id;
  visualization_msgs::Marker x_axis_marker = create_line_marker(++marker_id,std::make_tuple(1.0, 0.0, 0.0, 1.0));
  visualization_msgs::Marker y_axis_marker = create_line_marker(++marker_id,std::make_tuple(0.0, 1.0, 0.0, 1.0));
  visualization_msgs::Marker z_axis_marker = create_line_marker(++marker_id,std::make_tuple(0.0, 0.0, 1.0, 1.0));

  auto add_axis_line = [](const Isometry3d& eigen_pose, const Vector3d& dir,
      const geometry_msgs::Point& p1, visualization_msgs::Marker& marker){

    geometry_msgs::Point p2;
    Eigen::Vector3d line_endpoint;

    // axis endpoint
    line_endpoint = eigen_pose * dir;
    std::tie(p2.x, p2.y, p2.z)  = std::make_tuple(line_endpoint.x(), line_endpoint.y(), line_endpoint.z());

    // adding line
    marker.points.push_back(p1);
    marker.points.push_back(p2);
  };

  for(auto& poses : path)
  {
    for(auto& pose : poses.poses)
    {
      Eigen::Isometry3d eigen_pose;
      tf::poseMsgToEigen(pose,eigen_pose);

      geometry_msgs::Point p1;
      std::tie(p1.x, p1.y, p1.z) = std::make_tuple(pose.position.x, pose.position.y, pose.position.z);

      add_axis_line(eigen_pose,Vector3d::UnitX()* axis_length,p1, x_axis_marker);
      add_axis_line(eigen_pose,Vector3d::UnitY()* axis_length,p1, y_axis_marker);
      add_axis_line(eigen_pose,Vector3d::UnitZ()* axis_length,p1, z_axis_marker);
    }

  }

  markers.markers.push_back(x_axis_marker);
  markers.markers.push_back(y_axis_marker);
  markers.markers.push_back(z_axis_marker);
  return std::move(markers);
}

visualization_msgs::MarkerArray convertToDottedLineMarker(const noether_msgs::ToolRasterPath& toolpath,
                                                    const std::string& frame_id,
                                                    const std::string& ns,
                                                    const std::size_t start_id,
                                                    const std::tuple<float, float, float, float, float, float>& offset,
                                                    const float line_width,
                                                    const float point_size)
{
  return convertToDottedLineMarker(toolpath.paths,frame_id, ns, start_id, offset, line_width, point_size);
}

visualization_msgs::MarkerArray convertToDottedLineMarker(const std::vector<geometry_msgs::PoseArray>& path,
                                                    const std::string& frame_id,
                                                    const std::string& ns,
                                                    const std::size_t start_id,
                                                    const std::tuple<float, float, float, float, float, float>& offset,
                                                    const float line_width,
                                                    const float point_size)
{
  visualization_msgs::MarkerArray markers_msgs;
  visualization_msgs::Marker line_marker, points_marker;

  // configure line marker
  line_marker.action = line_marker.ADD;
  std::tie(line_marker.color.r, line_marker.color.g, line_marker.color.b, line_marker.color.a) = std::make_tuple(1.0, 1.0, 0.2, 1.0);
  line_marker.header.frame_id = frame_id;
  line_marker.type = line_marker.LINE_STRIP;
  line_marker.id = start_id;
  line_marker.lifetime = ros::Duration(0);
  line_marker.ns = ns;
  std::tie(line_marker.scale.x, line_marker.scale.y, line_marker.scale.z) = std::make_tuple(line_width, 0.0, 0.0);
  line_marker.pose = pose3DtoPoseMsg(offset);

  // configure point marker
  points_marker = line_marker;
  points_marker.type = points_marker.POINTS;
  points_marker.ns = ns;
  std::tie(points_marker.color.r, points_marker.color.g, points_marker.color.b, points_marker.color.a) = std::make_tuple(0.1, .8, 0.2, 1.0);
  std::tie(points_marker.scale.x, points_marker.scale.y, points_marker.scale.z) = std::make_tuple(point_size,point_size,point_size);

  int id_counter = start_id;
  for(auto& poses : path)
  {
    line_marker.points.clear();
    points_marker.points.clear();
    line_marker.points.reserve(poses.poses.size());
    points_marker.points.reserve(poses.poses.size());
    for(auto& pose : poses.poses)
    {
      geometry_msgs::Point p;
      std::tie(p.x, p.y, p.z) = std::make_tuple(pose.position.x, pose.position.y, pose.position.z);
      line_marker.points.push_back(p);
      points_marker.points.push_back(p);
    }

    line_marker.id = (++id_counter);
    points_marker.id = (++id_counter);
    markers_msgs.markers.push_back(line_marker);
    markers_msgs.markers.push_back(points_marker);
  }

  return markers_msgs;
}

}
