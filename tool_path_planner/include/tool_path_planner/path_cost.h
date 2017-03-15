#ifndef PATH_COST_HPP
#define PATH_COST_HPP

#include "ceres/ceres.h"

class SprayModel
{
  SprayModel(double sx, double sy, double cxy, double f)
    sx_(sx), sy_(sy), cxy_(cxy), f_(f)
  {
    sx2_ = sx_*sx_;
    sy2_ = sy_*sy_;
    one_minus_p2_ = 1.0 - cxy_*cxy_;
    multiplier_ = 2*3.14*sx_*sy_*sqrt(one_minus_p2);
    sxsy_ =sx_*sy_;
  };
  
  double sx_;       // variance in x
  double sy_;       // variance in y
  double cxy_;      // correlation between x and y
  double f_;        // effective focal length
  double sx2_,sy2_; // square of sx and sy
  double sxy_;      // sx*sy
  double multiplier_; // 1.0/(2*pi*sx*sy*sqrt(1-cxy*cxy)
  double one_minus_p2_; // 1 - cxy*cxy
};

class  PathPosesToSurfacePtCost
  {
  public:
    PathPosesToSurfacePtCost(SurfacPt pt, double dt, int num_poses, SprayModel SM) :
      pt_(pt), dt_(dt), SM_(SM), num_poses_(num_poses)
    {
    }
    template<typename T>
    bool operator()(const T* const pos_params,  /** 6DOF pose*/
		    T* residual) const
    {
      T paint_deposit = T(0);
      T angle_axis[3], position[3];
      T tool_point[3]; // point expressed in tool coordinates
      T u,v,z2;
      for(int i=0; i<num_poses_;i++){
	angle_axis[0] = pose_params[i];
	angle_axis[1] = pose_params[i+1];
	angle_axis[2] = pose_params[i+2];
	position[0]   = pose_params[i+3];
	position[1]   = pose_params[i+4];
	position[2]   = pose_params[i+5];
	ceres::AngleAxisRotatePoint(angle_axis, point, tool_point);
	
	u  = SM.f*tool_point[0]/tool_point[2]; // project point into the u-v plane like a pinhole camera
	v  = SM.f*tool_point[1]/tool_point[2]; 
	z2 = tool_point[2]*tool_point[2];
      
	paint_deposit += multiplier_*exp(-1.0/(2*one_minus_p2)*(u*u/sx2_ + v*v/sy2_ - 2*cxy_*u*v/cxy_))/z2;  
      }
      *residual = dt_ - paint_deposit;
      return true;
    } /** end of operator() */
    /** Factory to hide the construction of the CostFunction object from */
    /** the client code. */
    static ceres::CostFunction* Create(SurfacePt pt)
    {
      return (new ceres::AutoDiffCostFunction<PathPoseToSurfacePtCost, 1, 1, num_poses_*6>(new PathPoseToSurfacePtCost(pt)));
    }
    SurfacePt pt_; /** point on suface */
    double dt_; /** desired thickness of paint at this point */
    SprayModel SM_; /** spray model to determin paint deposited from a pose onto this point */
    int num_poses_; /** number of poses that can deposit paint at this point */
  }; 

#endif
