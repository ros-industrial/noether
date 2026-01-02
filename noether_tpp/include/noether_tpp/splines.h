#pragma once

#include <unsupported/Eigen/Splines>
#include <Eigen/Dense>

namespace noether
{
/**
 * @details Samples the input spline at the given spline parameter to create a 6-DoF pose, where the x-axis is aligned
 * with the first derivative of the spline.
 * @param spline
 * @param t Spline parameter (on [0, 1]) at which to sample the spline
 * @param ref_z Reference z-axis for use in computing the pose orientation
 * @return
 */
Eigen::Isometry3d sample(const Eigen::Spline3d& spline, const double t, const Eigen::Vector3d& ref_z);

/**
 * @details Computes the lengths (m) of each polynomial span of the spline. This method applies Simpson's composite 3/8
 * rule to numerically integrate the first derivative of each polynomial segment of the spline to compute the distance
 * @param spline
 * @return Vector of lengths (m) of each polynomial span of the spline (size `n_knots - 1`)
 */
std::vector<double> computeSpanLengths(const Eigen::Spline3d& spline);

/**
 * @details Computes the distances (m) along the spline to each knot. This method performs a cumulative sum of the span
 * lengths (given by computeSpanLengths) to return the total distance (m) along the spline to each knot
 * @param spline
 * @return Vector of distances (m) to each knot in the spline (size `n_knots`)
 */
std::vector<double> computeKnotDistances(const Eigen::Spline3d& spline);

/**
 * @details Computes the length (m) of the input spline. This method sums the length of all polynomial spans of the
 * spline, given by computeSpanLengths
 * @param spline
 * @return
 */
double computeLength(const Eigen::Spline3d& spline);

/**
 * @details Computes the spline parameter at a specific distance along on the spline
 * @param spline
 * @param dist Distance (m) along spline. When this value is negative, or greater than the length of the spline the
 * first/last polynomial segment will be extrapolated.
 * @param tol Error tolerance (m) with which to calculate the distance
 * @return
 */
double computeSplineParameterAtDistance(const Eigen::Spline3d& spline, const double dist, const double tol = 1.0e-4);

/**
 * @details Computes equidistant knot points along a spline. If the endpoints of the spline are included, the remaining
 * equally spaced points will be centered in between them.
 * @param spline
 * @param knot_spacing Spacing (m) between knots on the spline
 * @param include_endpoints Flag to indicate that the spline parameters for the endpoints of the spline (i.e., 0 and 1)
 * should be included in the output
 * @param tol Error tolerance (m) for calculating the distance along the spline
 * @return Vector of spline parameters that are equally spaced in distance along the spline
 */
std::vector<double> computeEquidistantKnots(const Eigen::Spline3d& spline,
                                            const double knot_spacing,
                                            const bool include_endpoints = false,
                                            const double tol = 1.0e-4);

}  //  namespace noether
