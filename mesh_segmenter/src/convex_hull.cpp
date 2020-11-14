#include <mesh_segmenter/convex_hull.h>

ConvexHullGenerator::ConvexHullGenerator() {}

bool ConvexHullGenerator::MakeMesh(const std::string& input, pcl::PointCloud<pcl::PointXYZ>& inMesh)
{
  // expects a .ply file
  pcl::PLYReader reader_;
  if (!reader_.read(input, inMesh))  // populate inMesh
  {
    printf("failed to read file");
    return false;
  }
  return true;
}

void ConvexHullGenerator::CleanMesh(const pcl::PointCloud<pcl::PointXYZ>& outMesh, pcl::PolygonMesh& outMeshPoly)
{
  // find centroid coords by finding average x, y, z

  Eigen::Matrix<float, 4, 1> mid;
  int centroid_success = pcl::compute3DCentroid(outMesh, mid);
  if (centroid_success == 0)
  {
    printf("Input cloud invalid");
    return;
  }

  Eigen::Vector3d midVec = { mid[0], mid[1], mid[2] };

  // invert bad polygons --------------------------------
  for (int t = 0; t < outMeshPoly.polygons.size(); t++)
  {
    pcl::Vertices verts;
    verts = outMeshPoly.polygons[t];

    pcl::PointXYZ a = outMesh.points[verts.vertices[0]];
    pcl::PointXYZ b = outMesh.points[verts.vertices[1]];
    pcl::PointXYZ c = outMesh.points[verts.vertices[2]];

    Eigen::Vector3d p0 = { a.x, a.y, a.z };
    Eigen::Vector3d p1 = { b.x, b.y, b.z };
    Eigen::Vector3d p2 = { c.x, c.y, c.z };

    Eigen::Vector3d v1 = p1 - p0;
    Eigen::Vector3d v2 = p2 - p0;
    Eigen::Vector3d d = p0 - midVec;
    Eigen::Vector3d normal = v1.cross(v2);
    float angle_threshold = d.dot(normal);

    if (angle_threshold < 0)
    {
      int temp;
      temp = verts.vertices[1];
      verts.vertices[1] = verts.vertices[2];
      verts.vertices[2] = temp;
    }
    outMeshPoly.polygons[t] = verts;
  }
  return;
}

bool ConvexHullGenerator::SaveMesh(const pcl::PointCloud<pcl::PointXYZ>& outMesh,
                                   pcl::PolygonMesh& outMeshPoly,
                                   const std::string& outfile)
{
  pcl::toPCLPointCloud2(outMesh, outMeshPoly.cloud);
  pcl::io::savePolygonFile(outfile, outMeshPoly, false);
  printf("Convex hill written to %s", outfile.c_str());
  return true;
}

bool ConvexHullGenerator::Generate(const std::string& infile, const std::string& outfile)
{
  pcl::PointCloud<pcl::PointXYZ> inMesh;
  pcl::PointCloud<pcl::PointXYZ> outMesh;
  pcl::PolygonMesh outMeshPoly;
  pcl::ConvexHull<pcl::PointXYZ> chull;

  if (ConvexHullGenerator::MakeMesh(infile, inMesh) == false)
  {
    printf("File read failed. Aborting");
    return false;
  }

  chull.setInputCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>(inMesh)));  // generate
                                                                                                         // hull
  chull.reconstruct(outMesh, outMeshPoly.polygons);  // save to outMesh

  ConvexHullGenerator::CleanMesh(outMesh, outMeshPoly);
  bool success = ConvexHullGenerator::SaveMesh(outMesh, outMeshPoly, outfile);

  return success;
}
