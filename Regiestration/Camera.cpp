#include "Camera.h"

#include <glog/logging.h>
#include "util/rotation.h"

#include "util/rq_decomposition.h"

using Eigen::AngleAxisd;
using Eigen::Map;
using Eigen::Matrix;
using Eigen::Matrix3d;
using Eigen::Vector2d;
using Eigen::Vector3d;
using Eigen::Vector4d;

namespace{

// Extract intrinsics from a calibration matrix of the following form:
//   [focal_length            skew               principal_point_x]
//   [0            focal_length * aspect_ratio   principal_point_y]
//   [0                         0                              1.0]
void IntrinsicsToCalibrationMatrix(const double focal_length,
	const double skew,
	const double aspect_ratio,
	const double principal_point_x,
	const double principal_point_y,
	Eigen::Matrix3d* calibration_matrix);

void CalibrationMatrixToIntrinsics(const Eigen::Matrix3d& calibration_matrix,
	double* focal_length,
	double* skew,
	double* aspect_ratio,
	double* principal_point_x,
	double* principal_point_y);

bool DecomposeProjectionMatrix(const Matrix3x4d pmatrix,
	Eigen::Matrix3d* calibration_matrix,
	Eigen::Vector3d* rotation,
	Eigen::Vector3d* position);

bool ComposeProjectionMatrix(const Eigen::Matrix3d& calibration_matrix,
	const Eigen::Vector3d& rotation,
	const Eigen::Vector3d& position,
	Matrix3x4d* pmatrix);


// implementation
void IntrinsicsToCalibrationMatrix(const double focal_length,
	const double skew,
	const double aspect_ratio,
	const double principal_point_x,
	const double principal_point_y,
	Eigen::Matrix3d* calibration_matrix)
{
	*calibration_matrix <<
		focal_length, skew, principal_point_x,
		0, focal_length * aspect_ratio, principal_point_y,
		0, 0, 1.0;
}

void CalibrationMatrixToIntrinsics(const Matrix3d& calibration_matrix,
	double* focal_length,
	double* skew,
	double* aspect_ratio,
	double* principal_point_x,
	double* principal_point_y) 
{
	CHECK_NE(calibration_matrix(2, 2), 0);
	*focal_length = calibration_matrix(0, 0) / calibration_matrix(2, 2);
	*skew = calibration_matrix(0, 1) / calibration_matrix(2, 2);
	*aspect_ratio = calibration_matrix(1, 1) / calibration_matrix(0, 0);
	*principal_point_x = calibration_matrix(0, 2) / calibration_matrix(2, 2);
	*principal_point_y = calibration_matrix(1, 2) / calibration_matrix(2, 2);
}


// Projects a 3x3 matrix to the rotation matrix in SO3 space with the closest
// Frobenius norm. For a matrix with an SVD decomposition M = USV, the nearest
// rotation matrix is R = UV'.
Matrix3d ProjectToRotationMatrix(const Matrix3d& matrix) {
	Eigen::JacobiSVD<Matrix3d> svd(matrix,
		Eigen::ComputeFullU | Eigen::ComputeFullV);
	Matrix3d rotation_mat = svd.matrixU() * (svd.matrixV().transpose());

	// The above projection will give a matrix with a determinant +1 or -1. Valid
	// rotation matrices have a determinant of +1.
	if (rotation_mat.determinant() < 0) {
		rotation_mat *= -1.0;
	}

	return rotation_mat;
}

bool DecomposeProjectionMatrix(const Matrix3x4d pmatrix,
	Matrix3d* calibration_matrix,
	Vector3d* rotation,
	Vector3d* position)
{
	theia::RQDecomposition<Matrix3d> rq(pmatrix.block<3, 3>(0, 0));

	Matrix3d rotation_matrix = ProjectToRotationMatrix(rq.matrixQ());


	const double k_det = rq.matrixR().determinant();
	if (k_det == 0) {
		return false;
	}

	Matrix3d& kmatrix = *calibration_matrix;
	if (k_det > 0) {
		kmatrix = rq.matrixR();
	}
	else {
		kmatrix = -rq.matrixR();
	}

	// Fix the matrix such that all internal parameters are greater than 0.
	for (int i = 0; i < 3; ++i) {
		if (kmatrix(i, i) < 0) {
			kmatrix.col(i) *= -1.0;
			rotation_matrix.row(i) *= -1.0;
		}
	}

	// Solve for t.
	const Vector3d t =
		kmatrix.triangularView<Eigen::Upper>().solve(pmatrix.col(3));

	// c = - R' * t, and flip the sign according to k_det;
	if (k_det > 0) {
		*position = -rotation_matrix.transpose() * t;
	}
	else {
		*position = rotation_matrix.transpose() * t;
	}

	const Eigen::AngleAxisd rotation_aa(rotation_matrix);
	*rotation = rotation_aa.angle() * rotation_aa.axis();

	return true;

}


bool ComposeProjectionMatrix(const Matrix3d& calibration_matrix,
	const Vector3d& rotation,
	const Vector3d& position,
	Matrix3x4d* pmatrix) 
{
	const double rotation_angle = rotation.norm();
	if (rotation_angle == 0) {
		pmatrix->block<3, 3>(0, 0) = Matrix3d::Identity();
	}
	else {
		pmatrix->block<3, 3>(0, 0) = Eigen::AngleAxisd(
			rotation_angle, rotation / rotation_angle).toRotationMatrix();
	}

	pmatrix->col(3) = -(pmatrix->block<3, 3>(0, 0) *  position);
	*pmatrix = calibration_matrix * (*pmatrix);
	return true;
}


// Projects a homogeneous 3D point to an image by assuming a camera model
// defined by the Camera class. This function is templated so that it can be
// used by Ceres for bundle adjument in addition to the standard reprojection
// with doubles.
//
// Returns the depth of the 3D point subject to the projective scale (i.e. the
// depth of the point assuming the image plane is at a depth of 1). The depth is
// useful, for instance, to determine if the point reprojects behind the image.
//
// NOTE: The unit test for this method is included in
// theia/sfm/camera/camera_test.cc
template <typename T>
T ProjectPointToImage(const T* extrinsic_parameters,
	const T* intrinsic_parameters,
	const T* point,
	T* pixel) {
	typedef Eigen::Matrix<T, 3, 1> Matrix3T;
	typedef Eigen::Map<const Matrix3T> ConstMap3T;

	// Remove the translation.
	Eigen::Matrix<T, 3, 1> adjusted_point =
		ConstMap3T(point) -
		point[3] * ConstMap3T(extrinsic_parameters + CCamera::POSITION);

	// Rotate the point.
	T rotated_point[3];
	ceres::AngleAxisRotatePoint(extrinsic_parameters + CCamera::ORIENTATION,
		adjusted_point.data(),
		rotated_point);

	// Get normalized pixel projection at image plane depth = 1.
	const T& depth = rotated_point[2];
	const T normalized_pixel[2] = { rotated_point[0] / depth,
		rotated_point[1] / depth };

	// Apply radial distortion.
	//T distorted_pixel[2];
	//RadialDistortPoint(normalized_pixel[0],
	//	normalized_pixel[1],
	//	intrinsic_parameters[CCamera::RADIAL_DISTORTION_1],
	//	intrinsic_parameters[CCamera::RADIAL_DISTORTION_2],
	//	distorted_pixel,
	//	distorted_pixel + 1);

	// Apply calibration parameters to transform normalized units into pixels.
	const T& focal_length = intrinsic_parameters[CCamera::FOCAL_LENGTH];
	const T& skew = intrinsic_parameters[CCamera::SKEW];
	const T& aspect_ratio = intrinsic_parameters[CCamera::ASPECT_RATIO];
	const T& principal_point_x = intrinsic_parameters[CCamera::PRINCIPAL_POINT_X];
	const T& principal_point_y = intrinsic_parameters[CCamera::PRINCIPAL_POINT_Y];

	//pixel[0] = focal_length * distorted_pixel[0] + skew * distorted_pixel[1] +
	//	principal_point_x;
	//pixel[1] = focal_length * aspect_ratio * distorted_pixel[1] +
	//	principal_point_y;

	pixel[0] = focal_length * normalized_pixel[0] + skew * normalized_pixel[1] +
		principal_point_x;
	pixel[1] = focal_length * aspect_ratio * normalized_pixel[1] +
		principal_point_y;

	return depth / point[3];
}


}

CCamera::CCamera() {
	// Set rotation and position to zero (i.e. identity).
	Map<Matrix<double, 1, 6> >(mutable_extrinsics()).setZero();

	SetFocalLength(1.0);
	SetAspectRatio(1.0);
	SetSkew(0.0);
	SetPrincipalPoint(0.0, 0.0);
	SetRadialDistortion(0.0, 0.0);

	image_size_[0] = 0;
	image_size_[1] = 0;
}

bool CCamera::InitializeFromProjectionMatrix(
	const int image_width,
	const int image_height,
	const Matrix3x4d projection_matrix) {
	DCHECK_GT(image_width, 0);
	DCHECK_GT(image_height, 0);
	image_size_[0] = image_width;
	image_size_[1] = image_height;

	Vector3d orientation, position;
	Matrix3d calibration_matrix;
	DecomposeProjectionMatrix(projection_matrix,
		&calibration_matrix,
		&orientation,
		&position);

	Map<Vector3d>(mutable_extrinsics() + ORIENTATION) = orientation;
	Map<Vector3d>(mutable_extrinsics() + POSITION) = position;

	if (calibration_matrix(0, 0) == 0 || calibration_matrix(1, 1) == 0) {
		LOG(INFO) << "Cannot set focal lengths to zero!";
		return false;
	}

	CalibrationMatrixToIntrinsics(calibration_matrix,
		mutable_intrinsics() + FOCAL_LENGTH,
		mutable_intrinsics() + SKEW,
		mutable_intrinsics() + ASPECT_RATIO,
		mutable_intrinsics() + PRINCIPAL_POINT_X,
		mutable_intrinsics() + PRINCIPAL_POINT_Y);
	return true;
}

void CCamera::GetProjectionMatrix(Matrix3x4d* pmatrix) const {
	Matrix3d calibration_matrix;
	GetCalibrationMatrix(&calibration_matrix);
	ComposeProjectionMatrix(calibration_matrix,
		GetOrientationAsAngleAxis(),
		GetPosition(),
		pmatrix);
}

void CCamera::GetCalibrationMatrix(Matrix3d* kmatrix) const {
	IntrinsicsToCalibrationMatrix(FocalLength(),
		Skew(),
		AspectRatio(),
		PrincipalPointX(),
		PrincipalPointY(),
		kmatrix);
}

double CCamera::ProjectPoint(const Vector4d& point, Vector2d* pixel) const 
{
	return ProjectPointToImage(extrinsics(),
		intrinsics(),
		point.data(),
		pixel->data());

}

Vector3d CCamera::PixelToUnitDepthRay(const Vector2d& pixel) const {
	Vector3d direction;

	// First, undo the calibration.
	const double focal_length_y = FocalLength() * AspectRatio();
	const double y_normalized = (pixel[1] - PrincipalPointY()) / focal_length_y;
	const double x_normalized =
		(pixel[0] - PrincipalPointX() - y_normalized * Skew()) / FocalLength();

	// Undo radial distortion.
	const Vector2d normalized_point(x_normalized, y_normalized);

	//Vector2d undistorted_point;
	//RadialUndistortPoint(normalized_point,
	//	RadialDistortion1(),
	//	RadialDistortion2(),
	//	&undistorted_point);

	// Apply rotation.
	const Matrix3d& rotation = GetOrientationAsRotationMatrix();
	
	//direction = rotation.transpose() * undistorted_point.homogeneous();
	direction = rotation.transpose() * normalized_point.homogeneous();

	return direction;
}

// ----------------------- Getter and Setter methods ---------------------- //
void CCamera::SetPosition(const Vector3d& position) {
	Map<Vector3d>(mutable_extrinsics() + POSITION) = position;
}

Vector3d CCamera::GetPosition() const {
	return Map<const Vector3d>(extrinsics() + POSITION);
}

void CCamera::SetOrientationFromRotationMatrix(const Matrix3d& rotation) {
	ceres::RotationMatrixToAngleAxis(
		ceres::ColumnMajorAdapter3x3(rotation.data()),
		mutable_extrinsics() + ORIENTATION);
}

void CCamera::SetOrientationFromAngleAxis(const Vector3d& angle_axis) {
	Map<Vector3d>(mutable_extrinsics() + ORIENTATION) = angle_axis;
}

Matrix3d CCamera::GetOrientationAsRotationMatrix() const {
	Matrix3d rotation;
	ceres::AngleAxisToRotationMatrix(
		extrinsics() + ORIENTATION,
		ceres::ColumnMajorAdapter3x3(rotation.data()));
	return rotation;
}

Vector3d CCamera::GetOrientationAsAngleAxis() const {
	return Map<const Vector3d>(extrinsics() + ORIENTATION);
}

void CCamera::SetFocalLength(const double focal_length) {
	mutable_intrinsics()[FOCAL_LENGTH] = focal_length;
}

double CCamera::FocalLength() const {
	return intrinsics()[FOCAL_LENGTH];
}

void CCamera::SetAspectRatio(const double aspect_ratio) {
	mutable_intrinsics()[ASPECT_RATIO] = aspect_ratio;
}
double CCamera::AspectRatio() const {
	return intrinsics()[ASPECT_RATIO];
}

void CCamera::SetSkew(const double skew) {
	mutable_intrinsics()[SKEW] = skew;
}

double CCamera::Skew() const {
	return intrinsics()[SKEW];
}

void CCamera::SetPrincipalPoint(const double principal_point_x,
	const double principal_point_y) {
	mutable_intrinsics()[PRINCIPAL_POINT_X] = principal_point_x;
	mutable_intrinsics()[PRINCIPAL_POINT_Y] = principal_point_y;
}

double CCamera::PrincipalPointX() const {
	return intrinsics()[PRINCIPAL_POINT_X];
}

double CCamera::PrincipalPointY() const {
	return intrinsics()[PRINCIPAL_POINT_Y];
}

void CCamera::SetRadialDistortion(const double radial_distortion_1,
	const double radial_distortion_2) {
	mutable_intrinsics()[RADIAL_DISTORTION_1] = radial_distortion_1;
	mutable_intrinsics()[RADIAL_DISTORTION_2] = radial_distortion_2;
}

double CCamera::RadialDistortion1() const {
	return intrinsics()[RADIAL_DISTORTION_1];
}

double CCamera::RadialDistortion2() const {
	return intrinsics()[RADIAL_DISTORTION_2];
}

void CCamera::SetImageSize(const int image_width, const int image_height) {
	image_size_[0] = image_width;
	image_size_[1] = image_height;
}