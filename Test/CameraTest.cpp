#include "CameraTest.h"

#include <glog/logging.h>
#include <Eigen/Dense>

#include "Camera.h"
#include "TestUitls.h"
#include "util/rotation.h"

namespace{

	void ReprojectionTest(const CCamera& camera) 
	{
		const double kTolerance = 1e-5;

		for (int i = 0; i < 10; i++) 
		{
			// Get a random pixel within the image.
			const Eigen::Vector2d pixel =
				camera.ImageWidth() * (Eigen::Vector2d::Random() + Eigen::Vector2d::Ones()) / 2.0;

			// Get the normalized ray of that pixel.
			const Eigen::Vector3d normalized_ray = camera.PixelToUnitDepthRay(pixel);

			const double random_depth = test::RandDouble(0.01, 100.0);
			const Eigen::Vector4d random_point =
				(camera.GetPosition() + normalized_ray * random_depth)
				.homogeneous();

			Eigen::Vector2d reprojected_pixel;
			const double depth =
				camera.ProjectPoint(random_point, &reprojected_pixel);

			// Expect the reconstructed 3d points to be close.
			CHECK_LT(std::abs(random_depth - depth),
				kTolerance * random_depth)
				<< "real depth = " << random_depth
				<< " and reconstructed depth = " << depth;

			// Expect the reprojection to be close.
			CHECK_LT((pixel - reprojected_pixel).norm(), kTolerance)
				<< "gt pixel: " << pixel.transpose()
				<< "\nreprojected pixel: " << reprojected_pixel.transpose();
		}
	}
}

CCameraTest::CCameraTest()
{
}


CCameraTest::~CCameraTest()
{
}

bool CCameraTest::Test()
{
	CHECK(ProjectionMatrixTest());
	CHECK(InternalParameterGettersAndSetters());
	CHECK(ExternalParameterGettersAndSetters());
	CHECK(ReprojectionNoDistortion());

	return true;
}

bool CCameraTest::ProjectionMatrixTest()
{
	LOG(INFO);

	const double kTolerance = 1e-12;

	CCamera camera;
	const double image_size = 500;
	for (int i = 0; i < 100; i++) {
		const Matrix3x4d gt_projection_matrix = Matrix3x4d::Random();
		CHECK(camera.InitializeFromProjectionMatrix(image_size,
			image_size,
			gt_projection_matrix));
		Matrix3x4d projection_matrix;
		camera.GetProjectionMatrix(&projection_matrix);

		CHECK(test::ArraysEqualUpToScale(12,
			gt_projection_matrix.data(),
			projection_matrix.data(),
			kTolerance));
	}

	LOG(INFO);

	return true;
}

bool CCameraTest::InternalParameterGettersAndSetters() 
{
	LOG(INFO);

	CCamera camera;

	// Check that default values are set
	CHECK_EQ(camera.FocalLength(), 1.0);
	CHECK_EQ(camera.AspectRatio(), 1.0);
	CHECK_EQ(camera.Skew(), 0.0);
	CHECK_EQ(camera.PrincipalPointX(), 0.0);
	CHECK_EQ(camera.PrincipalPointY(), 0.0);
	CHECK_EQ(camera.RadialDistortion1(), 0.0);
	CHECK_EQ(camera.RadialDistortion2(), 0.0);

	// Set parameters to different values.
	camera.SetFocalLength(600.0);
	camera.SetAspectRatio(0.9);
	camera.SetSkew(0.01);
	camera.SetPrincipalPoint(300.0, 400.0);
	camera.SetRadialDistortion(0.01, 0.001);

	// Check that the values were updated.
	CHECK_EQ(camera.FocalLength(), 600.0);
	CHECK_EQ(camera.AspectRatio(), 0.9);
	CHECK_EQ(camera.Skew(), 0.01);
	CHECK_EQ(camera.PrincipalPointX(), 300.0);
	CHECK_EQ(camera.PrincipalPointY(), 400.0);
	CHECK_EQ(camera.RadialDistortion1(), 0.01);
	CHECK_EQ(camera.RadialDistortion2(), 0.001);

	LOG(INFO);

	return true;
}

bool CCameraTest::ExternalParameterGettersAndSetters()
{
	LOG(INFO);

	const double kTolerance = 1e-16;

	CCamera camera;

	// Check that the default values are set.
	CHECK_EQ(camera.GetPosition().squaredNorm(), 0.0);
	CHECK_EQ(camera.GetOrientationAsAngleAxis().squaredNorm(), 0.0);
	CHECK_EQ((camera.GetOrientationAsRotationMatrix() -
		Eigen::Matrix3d::Identity()).squaredNorm(),
		0.0);

	// Check that position getter/setters work.
	camera.SetPosition(Eigen::Vector3d::Ones());
	CHECK_EQ((camera.GetPosition() - Eigen::Vector3d::Ones()).squaredNorm(),
		0.0);

	// Check that angle axis getter/setters work.
	Eigen::Vector3d gt_angle_axis(1.0, 1.0, 1.0);
	gt_angle_axis = Eigen::Vector3d(0.3, 0.7, 0.4);
	Eigen::Matrix3d gt_rotation_matrix;
	ceres::AngleAxisToRotationMatrix(gt_angle_axis.data(),
		gt_rotation_matrix.data());
	camera.SetOrientationFromRotationMatrix(gt_rotation_matrix);
	CHECK_LE(
		(camera.GetOrientationAsAngleAxis() - gt_angle_axis).squaredNorm(),
		kTolerance);
	CHECK_LE((camera.GetOrientationAsRotationMatrix() - gt_rotation_matrix)
		.squaredNorm(),
		kTolerance);

	// Check that rotation matrix getter/setters work.
	gt_angle_axis = Eigen::Vector3d(0.3, 0.7, 0.4);
	ceres::AngleAxisToRotationMatrix(gt_angle_axis.data(),
		gt_rotation_matrix.data());
	camera.SetOrientationFromRotationMatrix(gt_rotation_matrix);
	CHECK_LE(
		(camera.GetOrientationAsAngleAxis() - gt_angle_axis).squaredNorm(),
		kTolerance);
	CHECK_LE((camera.GetOrientationAsRotationMatrix() - gt_rotation_matrix)
		.squaredNorm(),
		kTolerance);

	LOG(INFO);

	return true;
}



bool CCameraTest::ReprojectionNoDistortion()
{
	LOG(INFO);

	test::InitRandomGenerator();
	CCamera camera;
	const double image_size = 600;
	for (int i = 0; i < 1; i++) {
		// Initialize a random camera.
		camera.InitializeFromProjectionMatrix(image_size, image_size,
			Matrix3x4d::Random());

		// Initialize random positive radial distortion parameters.
		camera.SetRadialDistortion(0, 0);
		ReprojectionTest(camera);
	}
	LOG(INFO);

	return true;
}
