#ifndef __CAMERA_H__
#define __CAMERA_H__

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <vector>

#include "Types.h"

class CCamera
{
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	CCamera();
	~CCamera() {}

	// Initializes the camera intrinsic and extrinsic parameters from the
	// projection matrix by decomposing the matrix.
	//
	// NOTE: The projection matrix does not contain information about radial
	// distortion, so those parameters will need to be set separately.
	bool InitializeFromProjectionMatrix(
		const int image_width,
		const int image_height,
		const Matrix3x4d projection_matrix);

	// ---------------------------- Helper methods ---------------------------- //
	// Returns the projection matrix. Does not include radial distortion.
	void GetProjectionMatrix(Matrix3x4d* pmatrix) const;

	// Returns the calibration matrix in the form specified above.
	void GetCalibrationMatrix(Eigen::Matrix3d* kmatrix) const;

	// Projects the homogeneous 3D point into the image plane and undistorts the
	// point according to the radial distortion parameters. The function returns
	// the depth of the point so that points that project behind the camera (i.e.,
	// negative depth) can be determined. Points at infinity return a depth of
	// infinity.
	double ProjectPoint(const Eigen::Vector4d& point,
		Eigen::Vector2d* pixel) const;

	// Converts the pixel point to a ray in 3D space such that the origin of the
	// ray is at the camera center and the direction is the pixel direction
	// rotated according to the camera orientation in 3D space.
	//
	// NOTE: The depth of the ray is set to 1. This is so that we can remain
	// consistent with ProjectPoint. That is:
	//      if we have:
	//         d = ProjectPoint(X, &x);
	//         r = PixelToRay(x);
	//      then it will be the case that
	//         X = c + r * d;
	//    X is the 3D point
	//    x is the image projection of X
	//    c is the camera position
	//    r is the ray obtained from PixelToRay
	//    d is the depth of the 3D point with respect to the image
	Eigen::Vector3d PixelToUnitDepthRay(const Eigen::Vector2d& pixel) const;

	// ----------------------- Getter and Setter methods ---------------------- //
	void SetPosition(const Eigen::Vector3d& position);
	Eigen::Vector3d GetPosition() const;

	void SetOrientationFromRotationMatrix(const Eigen::Matrix3d& rotation);
	void SetOrientationFromAngleAxis(const Eigen::Vector3d& angle_axis);
	Eigen::Matrix3d GetOrientationAsRotationMatrix() const;
	Eigen::Vector3d GetOrientationAsAngleAxis() const;

	void SetFocalLength(const double focal_length);
	double FocalLength() const;

	void SetAspectRatio(const double aspect_ratio);
	double AspectRatio() const;

	void SetSkew(const double skew);
	double Skew() const;

	void SetPrincipalPoint(const double principal_point_x,
		const double principal_point_y);
	double PrincipalPointX() const;
	double PrincipalPointY() const;

	void SetRadialDistortion(const double radial_distortion_1,
		const double radial_distortion_2);
	double RadialDistortion1() const;
	double RadialDistortion2() const;

	void SetImageSize(const int image_width, const int image_height);
	int ImageWidth() const { return image_size_[0]; }
	int ImageHeight() const { return image_size_[1]; }

	const double* extrinsics() const { return camera_parameters_; }
	double* mutable_extrinsics() { return camera_parameters_; }

	const double* intrinsics() const {
		return camera_parameters_ + kExtrinsicsSize;
	}
	double* mutable_intrinsics() { return camera_parameters_ + kExtrinsicsSize; }

	const double* parameters() const { return camera_parameters_; }
	double* mutable_parameters() { return camera_parameters_; }

	// Indexing for the location of parameters. Collecting the extrinsics and
	// intrinsics into a single array makes the interface to bundle adjustment
	// with Ceres much simpler.
	enum ExternalParametersIndex {
		POSITION = 0,
		ORIENTATION = 3
	};

	enum InternalParametersIndex{
		FOCAL_LENGTH = 0,
		ASPECT_RATIO = 1,
		SKEW = 2,
		PRINCIPAL_POINT_X = 3,
		PRINCIPAL_POINT_Y = 4,
		RADIAL_DISTORTION_1 = 5,
		RADIAL_DISTORTION_2 = 6
	};

	static const int kExtrinsicsSize = 6;
	static const int kIntrinsicsSize = 7;
	static const int kParameterSize = kExtrinsicsSize + kIntrinsicsSize;

private:

	double camera_parameters_[kParameterSize];

	// The image size as width then height.
	int image_size_[2];

};

#endif // __CAMERA_H__
