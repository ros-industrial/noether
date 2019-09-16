#include <mesh_segmenter/convex_hull.h>

using namespace std;
ConvexHullGenerator::ConvexHullGenerator(){}  
void ConvexHull::makemesh(string input, pcl::PointCloud<pcl::PointXYZ> *inMesh)
{
//making mesh----------------------------------------------
  //expects a .ply file
  pcl::PLYReader Reader;
  Reader.read(input, *inMesh); //populate inMesh
  return;
}


void ConvexHullGenerator::cleanmesh(pcl::PointCloud<pcl::PointXYZ> outMesh. pcl::PolygonMesh::Ptr outMeshPoly)
{

  //find centroid coords by finding average x, y, z -----------------------

  Eigen::Matrix< float, 4, 1 > mid;
  int centroid_success = pcl::compute3DCentroid(*outMesh, mid); //this needs to be checked
  Eigen::Vector3d midVec= {mid[0], mid[1], mid[2]};

  //invert bad polygons --------------------------------
  for (int t=0; t < (outMeshPoly->polygons.size()); t++)
  {
    pcl::Vertices verts;
    verts = outMeshPoly->polygons[t];

    pcl::PointXYZ a = outMesh->points[verts.vertices[0]];
    pcl::PointXYZ b = outMesh->points[verts.vertices[1]];
    pcl::PointXYZ c = outMesh->points[verts.vertices[2]];


    Eigen::Vector3d p0 = {a.x, a.y, a.z};
    Eigen::Vector3d p1 = {b.x, b.y, b.z};
    Eigen::Vector3d p2 = {c.x, c.y, c.z};

    Eigen::Vector3d v1 = p1 - p0; 
    Eigen::Vector3d v2 = p2 - p0;
    Eigen::Vector3d d = p0 - midVec;
    Eigen::Vector3d normal = v1.cross(v2);
    float works = d.dot(normal);

    if (works < 0)
    {
      int temp;
      temp = verts.vertices[1];
      verts.vertices[1] = verts.vertices[2];
      verts.vertices[2] = temp;

    }
    outMeshPoly->polygons[t] = verts;
  } 
  return;
}

bool ConvexHullGenerator::savemesh(pcl::PointCloud<pcl::PointXYZ>::Ptr outMesh, pcl::PolygonMesh::Ptr outMeshPoly, string outfile)
{
  pcl::toPCLPointCloud2(*outMesh, outMeshPoly->cloud);
  pcl::io::savePolygonFile(outfile, *outMeshPoly, false);
  return true;
}

bool ConvexHullGenerator:: generate_ch(string infile,string outfile)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr inMesh (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr outMesh (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PolygonMesh::Ptr outMeshPoly (new pcl::PolygonMesh);
    pcl::ConvexHull<pcl::PointXYZ> chull;

    ConvexHullGenerator::makemesh(infile, inMesh); //theres like a 2% chance I'm passing this pointer correctly

    chull.setInputCloud(inMesh); //generate hull
    chull.reconstruct(*outMesh, outMeshPoly->polygons); //save to outMesh

    ConvexHullGenerator::cleanmesh(outMesh, outMeshPoly);
    bool success = ConvexHullGenerator::savemesh(outMesh, outMeshPoly, outfile);

  return success;
}
  



