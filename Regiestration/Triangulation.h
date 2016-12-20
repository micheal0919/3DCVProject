#ifndef __TRIANGULATION_H__
#define __TRIANGULATION_H__

#include <vector>
#include <Eigen/Core>

#include "Types.h"

bool Triangulate(const Matrix3x4d& pose1,
	const Matrix3x4d& pose2,
	const Eigen::Vector2d& point1,
	const Eigen::Vector2d& point2,
	Eigen::Vector4d* triangulated_point);

bool TriangulateDLT(const Matrix3x4d& pose1,
	const Matrix3x4d& pose2,
	const Eigen::Vector2d& point1,
	const Eigen::Vector2d& point2,
	Eigen::Vector4d* triangulated_point);

bool TriangulateMidpoint(const std::vector<Eigen::Vector3d>& origins,
	const std::vector<Eigen::Vector3d>& ray_directions,
	Eigen::Vector4d* triangulated_point);

bool TriangulateNViewSVD(
	const std::vector<Matrix3x4d>& poses,
	const std::vector<Eigen::Vector2d>& points,
	Eigen::Vector4d* triangulated_point);

bool TriangulateNView(const std::vector<Matrix3x4d>& poses,
	const std::vector<Eigen::Vector2d>& points,
	Eigen::Vector4d* triangulated_point);

bool IsTriangulatedPointInFrontOfCameras(
	const FeatureCorrespondence& correspondence,
	const Eigen::Matrix3d& rotation,
	const Eigen::Vector3d& position);

bool SufficientTriangulationAngle(
	const std::vector<Eigen::Vector3d>& ray_directions,
	const double min_triangulation_angle_degrees);


#endif // __TRIANGULATION_H__
