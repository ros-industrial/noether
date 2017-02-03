#include <noether_conversions/noether_conversions.h>
#include <Eigen/Geometry>
#include <tf/LinearMath/Matrix3x3.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/Transform.h>
#include <ros/time.h>
#include <vtkPointData.h>


std::vector<geometry_msgs::PoseArray> posesConvertVTKtoGeometryMsgs(const std::vector<tool_path_planner::ProcessPath>& paths)
{
  std::vector<geometry_msgs::PoseArray> poseArrayVector;
  for(int j = 0; j < paths.size(); ++j)
  {
     geometry_msgs::PoseArray poses;
     poses.header.seq = j;
     poses.header.stamp = ros::Time::now();
     poses.header.frame_id = "0";
     for(int k = 0; k < paths[j].line->GetPoints()->GetNumberOfPoints(); ++k)
     {
        geometry_msgs::Pose pose;
        double pt[3];

        // Get the point location
        paths[j].line->GetPoints()->GetPoint(k, pt);
        pose.position.x = pt[0];
        pose.position.y = pt[1];
        pose.position.z = pt[2];

        // Get the point normal and derivative for creating the 3x3 transform
        double* norm = paths[j].line->GetPointData()->GetNormals()->GetTuple(k);
        double* der = paths[j].derivatives->GetPointData()->GetNormals()->GetTuple(k);

        // perform cross product to get the third axis direction
        Eigen::Vector3d u(norm[0], norm[1], norm[2]);
        Eigen::Vector3d v(der[0], der[1], der[2]);
        Eigen::Vector3d w = u.cross(v);
        w.normalize();

        // after first cross product, u and w will be orthogonal.  Perform cross
        // product one more time to make sure that v is perfectly orthogonal to u and w
        v = u.cross(w);
        v.normalize();

        // set the matrix transform using u, v, and w
        tf::Matrix3x3 matrix;
        matrix.setValue(u[0], u[1], u[2], v[0], v[1], v[2], w[0], w[1], w[2]);
        tf::Quaternion q;
        matrix.getRotation(q);
        tf::quaternionTFToMsg(q, pose.orientation);

        // push back new matrix (pose and orientation), this makes one long vector
        // may need to break this up more
        poses.poses.push_back(pose);

        }
     poseArrayVector.push_back(poses);
    }

  return poseArrayVector;
}
