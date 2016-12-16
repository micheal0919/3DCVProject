#ifndef __TRIANGULATION_H__
#define __TRIANGULATION_H__

#include <vector>
#include <Eigen/Core>

#include "Types.h"

// Triangulates 2 posed views using the "Triangulation Made Easy" by Lindstrom
// (CVPR 2010)". The inputs are the projection matrices and image points. For
// two view reconstructions where only an essential matrix or fundamental matrix
// is available you can use ProjectionMatricesFromFundamentalMatrix in
// theia/sfm/pose/fundamental_matrix_utils.h
bool Triangulate(const Matrix3x4d& pose1,
	const Matrix3x4d& pose2,
	const Eigen::Vector2d& point1,
	const Eigen::Vector2d& point2,
	Eigen::Vector4d* triangulated_point);

// Triangulates 2 posed views using the DLT method from HZZ 12.2 p 312. The
// inputs are the projection matrices and the image observations. Returns true
// on success and false on failure.
bool TriangulateDLT(const Matrix3x4d& pose1,
	const Matrix3x4d& pose2,
	const Eigen::Vector2d& point1,
	const Eigen::Vector2d& point2,
	Eigen::Vector4d* triangulated_point);

// Triangulates a 3D point by determining the closest point between the
// rays. This method is known to be suboptimal in terms of reprojection error
// but it is extremely fast. We assume that the directions are unit vectors.
bool TriangulateMidpoint(const std::vector<Eigen::Vector3d>& origins,
	const std::vector<Eigen::Vector3d>& ray_directions,
	Eigen::Vector4d* triangulated_point);

// Computes n-view triangulation by computing the SVD that wil approximately
// minimize reprojection error. The inputs are the projection matrices and the
// image observations. Returns true on success and false on failure.
bool TriangulateNViewSVD(
	const std::vector<Matrix3x4d>& poses,
	const std::vector<Eigen::Vector2d>& points,
	Eigen::Vector4d* triangulated_point);

// Computes n-view triangulation by an efficient L2 minimization of the
// algebraic error. This minimization is independent of the number of points, so
// it is extremely scalable. It gives better reprojection errors in the results
// and is significantly faster. The inputs are the projection matrices and the
// image observations. Returns true on success and false on failure.
bool TriangulateNView(const std::vector<Matrix3x4d>& poses,
	const std::vector<Eigen::Vector2d>& points,
	Eigen::Vector4d* triangulated_point);

// Determines if the 3D point is in front of the camera or not. We can simply
// compute the homogeneous ray intersection (closest point to two rays) and
// determine if the depth of the point is positive for both camera.
bool IsTriangulatedPointInFrontOfCameras(
	const FeatureCorrespondence& correspondence,
	const Eigen::Matrix3d& rotation,
	const Eigen::Vector3d& position);

// Returns true if the triangulation angle between any two observations is
// sufficient.
bool SufficientTriangulationAngle(
	const std::vector<Eigen::Vector3d>& ray_directions,
	const double min_triangulation_angle_degrees);


#endif // __TRIANGULATION_H__
