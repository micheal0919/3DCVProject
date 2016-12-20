#include "Triangulation.h"

#include <Eigen/QR>
#include <Eigen/SVD>
#include <Eigen/Eigenvalues>
#include <Eigen/Geometry>

#include <glog/logging.h>

#include "util/uitl.h"

using Eigen::Matrix;
using Eigen::MatrixXd;
using Eigen::Matrix3d;
using Eigen::Matrix4d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;

namespace {

void FindOptimalImagePoints(const Matrix3d& ematrix,
	const Vector2d& point1,
	const Vector2d& point2,
	Vector2d* corrected_point1,
	Vector2d* corrected_point2) 
{
	const Vector3d point1_homog = point1.homogeneous();
	const Vector3d point2_homog = point2.homogeneous();

	// A helper matrix to isolate certain coordinates.
	Matrix<double, 2, 3> s_matrix;
	s_matrix <<
		1, 0, 0,
		0, 1, 0;

	const Eigen::Matrix2d e_submatrix = ematrix.topLeftCorner<2, 2>();

	// The epipolar line from one image point in the other image.
	Vector2d epipolar_line1 = s_matrix * ematrix * point2_homog;
	Vector2d epipolar_line2 = s_matrix * ematrix.transpose() * point1_homog;

	const double a = epipolar_line1.transpose() * e_submatrix * epipolar_line2;
	const double b =
		(epipolar_line1.squaredNorm() + epipolar_line2.squaredNorm()) / 2.0;
	const double c = point1_homog.transpose() * ematrix * point2_homog;

	const double d = sqrt(b * b - a * c);

	double lambda = c / (b + d);
	epipolar_line1 -= e_submatrix * lambda * epipolar_line1;
	epipolar_line2 -= e_submatrix.transpose() * lambda * epipolar_line2;

	lambda *=
		(2.0 * d) / (epipolar_line1.squaredNorm() + epipolar_line2.squaredNorm());

	*corrected_point1 = (point1_homog - s_matrix.transpose() * lambda *
		epipolar_line1).hnormalized();
	*corrected_point2 = (point2_homog - s_matrix.transpose() * lambda *
		epipolar_line2).hnormalized();
}

}  // namespace

// Triangulates 2 posed views
bool Triangulate(const Matrix3x4d& pose1,
	const Matrix3x4d& pose2,
	const Vector2d& point1,
	const Vector2d& point2,
	Vector4d* triangulated_point) 
{
	return true;
	//Matrix3d fmatrix;
	//FundamentalMatrixFromProjectionMatrices(pose1.data(),
	//	pose2.data(),
	//	fmatrix.data());


	// TODO

	//Vector2d corrected_point1, corrected_point2;
	//FindOptimalImagePoints(fmatrix, point1, point2,
	//	&corrected_point1, &corrected_point2);

	//// Now the two points are guaranteed to intersect. We can use the DLT method
	//// since it is easy to construct.
	//return TriangulateDLT(pose1,
	//	pose2,
	//	corrected_point1,
	//	corrected_point2,
	//	triangulated_point);
}

// Triangulates a 3D point by determining the closest point between the two
// rays. This method is known to be suboptimal in terms of reprojection error
// but it is extremely fast.
bool TriangulateMidpoint(const std::vector<Vector3d>& ray_origin,
	const std::vector<Vector3d>& ray_direction,
	Eigen::Vector4d* triangulated_point) 
{
	CHECK_NOTNULL(triangulated_point);
	CHECK_GE(ray_origin.size(), 2);
	CHECK_EQ(ray_origin.size(), ray_direction.size());

	Eigen::Matrix4d A;
	A.setZero();
	Eigen::Vector4d b;
	b.setZero();
	for (int i = 0; i < ray_origin.size(); i++) {
		const Eigen::Vector4d ray_direction_homog(ray_direction[i].x(),
			ray_direction[i].y(),
			ray_direction[i].z(),
			0);
		const Eigen::Matrix4d A_term =
			Eigen::Matrix4d::Identity() -
			ray_direction_homog * ray_direction_homog.transpose();
		A += A_term;
		b += A_term * ray_origin[i].homogeneous();
	}

	Eigen::ColPivHouseholderQR<Eigen::Matrix4d> qr(A);
	if (qr.info() != Eigen::Success) {
		return false;
	}
	*triangulated_point = qr.solve(b);
	return qr.info() == Eigen::Success;
}

// Triangulates 2 posed views
bool TriangulateDLT(const Matrix3x4d& pose1,
	const Matrix3x4d& pose2,
	const Vector2d& point1,
	const Vector2d& point2,
	Vector4d* triangulated_point) 
{
	return true;
	//Matrix4d design_matrix;
	//design_matrix.row(0) = point1[0] * pose1.row(2) - pose1.row(0);
	//design_matrix.row(1) = point1[1] * pose1.row(2) - pose1.row(1);
	//design_matrix.row(2) = point2[0] * pose2.row(2) - pose2.row(0);
	//design_matrix.row(3) = point2[1] * pose2.row(2) - pose2.row(1);

	//// Extract nullspace.
	//*triangulated_point =
	//	design_matrix.jacobiSvd(Eigen::ComputeFullV).matrixV().rightCols<1>();
	//return true;
}

// Triangulates N views by computing SVD that minimizes the error.
bool TriangulateNViewSVD(const std::vector<Matrix3x4d>& poses,
	const std::vector<Vector2d>& points,
	Vector4d* triangulated_point) 
{
	CHECK_EQ(poses.size(), points.size());

	MatrixXd design_matrix(3 * points.size(), 4 + points.size());

	for (int i = 0; i < points.size(); i++) {
		design_matrix.block<3, 4>(3 * i, 0) = -poses[i].matrix();
		design_matrix.block<3, 1>(3 * i, 4 + i) = points[i].homogeneous();
	}

	*triangulated_point = design_matrix.jacobiSvd(Eigen::ComputeFullV)
		.matrixV()
		.rightCols<1>()
		.head(4);
	return true;
}

bool TriangulateNView(const std::vector<Matrix3x4d>& poses,
	const std::vector<Vector2d>& points,
	Vector4d* triangulated_point) 
{
	CHECK_EQ(poses.size(), points.size());

	Matrix4d design_matrix = Matrix4d::Zero();
	for (int i = 0; i < points.size(); i++) {
		const Vector3d norm_point = points[i].homogeneous().normalized();
		const Eigen::Matrix<double, 3, 4> cost_term =
			poses[i].matrix() -
			norm_point * norm_point.transpose() * poses[i].matrix();
		design_matrix = design_matrix + cost_term.transpose() * cost_term;
	}

	Eigen::SelfAdjointEigenSolver<Matrix4d> eigen_solver(design_matrix);
	*triangulated_point = eigen_solver.eigenvectors().col(0);
	return eigen_solver.info() == Eigen::Success;
}

bool IsTriangulatedPointInFrontOfCameras(
	const FeatureCorrespondence& correspondence,
	const Matrix3d& rotation,
	const Vector3d& position) 
{
	//const Vector3d dir1 = correspondence.feature1.homogeneous();
	//const Vector3d dir2 =
	//	rotation.transpose() * correspondence.feature2.homogeneous();

	Eigen::Vector2d f1(correspondence.feature1.x, correspondence.feature1.y);
	Eigen::Vector2d f2(correspondence.feature2.x, correspondence.feature2.y);
	const Vector3d dir1 = f1.homogeneous();
	const Vector3d dir2 = rotation.transpose() * f2.homogeneous();

	const double dir1_sq = dir1.squaredNorm();
	const double dir2_sq = dir2.squaredNorm();
	const double dir1_dir2 = dir1.dot(dir2);
	const double dir1_pos = dir1.dot(position);
	const double dir2_pos = dir2.dot(position);

	return (dir2_sq * dir1_pos - dir1_dir2 * dir2_pos > 0 &&
		dir1_dir2 * dir1_pos - dir1_sq * dir2_pos > 0);
}


// Returns true if the triangulation angle between any two observations is
// sufficient.
bool SufficientTriangulationAngle(
	const std::vector<Eigen::Vector3d>& ray_directions,
	const double min_triangulation_angle_degrees) 
{
	// Test that the angle between the rays is sufficient.
	const double cos_of_min_angle =
		cos(DegToRad(min_triangulation_angle_degrees));
	for (int i = 0; i < ray_directions.size(); i++) {
		for (int j = i + 1; j < ray_directions.size(); j++) {
			if (ray_directions[i].dot(ray_directions[j]) < cos_of_min_angle) {
				return true;
			}
		}
	}
	return false;
}
