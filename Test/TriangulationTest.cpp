#include "TriangulationTest.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glog/logging.h>

#include "Triangulation.h"
#include "TestUitls.h"
#include "util/uitl.h"

namespace
{
	enum TriangulationType 
	{
		STANDARD = 1,
		DLT = 2,
		MIDPOINT = 3
	};

	double ReprojectionError(const Matrix3x4d& pose,
		const Eigen::Vector4d& world_point,
		const Eigen::Vector2d& image_point)
	{
		const Eigen::Vector3d reprojected_point = pose * world_point;
		const double sq_reproj_error =
			(reprojected_point.hnormalized() - image_point).squaredNorm();
		return sq_reproj_error;
	}

	void TestTriangulationBasic(const TriangulationType type,
		const Eigen::Vector3d& point_3d,
		const Eigen::Quaterniond& rel_rotation,
		const Eigen::Vector3d& rel_translation,
		const double projection_noise,
		const double max_reprojection_error) 
	{
		test::InitRandomGenerator();

		Matrix3x4d pose1;
		pose1 <<
			rel_rotation.toRotationMatrix(), rel_translation.normalized();
		const Matrix3x4d pose2 = Matrix3x4d::Identity();

		// Reproject point into both image 2, assume image 1 is identity rotation at
		// the origin.
		Eigen::Vector2d image_point1 =
			(pose1 * point_3d.homogeneous()).eval().hnormalized();
		Eigen::Vector2d image_point2 =
			(pose2 * point_3d.homogeneous()).eval().hnormalized();

		// Add projection noise if required.
		if (projection_noise) {
			test::AddNoiseToProjection(projection_noise, &image_point1);
			test::AddNoiseToProjection(projection_noise, &image_point2);
		}

		// Triangulate with Optimal.
		Eigen::Vector4d triangulated_point;
		if (type == STANDARD) {
			CHECK(Triangulate(pose1, pose2,
				image_point1, image_point2,
				&triangulated_point));
		}
		else if (type == DLT) {
			CHECK(
				TriangulateDLT(pose1, pose2,
				image_point1, image_point2,
				&triangulated_point));
		}
		else if (type == MIDPOINT) {
			std::vector<Eigen::Vector3d> origins;
			std::vector<Eigen::Vector3d> directions;

			const Eigen::Matrix3d rotation1 = pose1.block<3, 3>(0, 0);
			origins.emplace_back(-rotation1.transpose() * pose1.col(3));
			directions.emplace_back(
				(rotation1.transpose() * image_point1.homogeneous()).normalized());
			const Eigen::Matrix3d rotation2 = pose2.block<3, 3>(0, 0);
			origins.emplace_back(-rotation2.transpose() * pose2.col(3));
			directions.emplace_back(
				(rotation2.transpose() * image_point2.homogeneous()).normalized());
			CHECK(
				TriangulateMidpoint(origins, directions, &triangulated_point));
		}
		else {
			LOG(ERROR) << "Incompatible Triangulation type!";
		}

		// Check the reprojection error.
		CHECK_LE(
			ReprojectionError(pose1, triangulated_point, image_point1),
			max_reprojection_error);
		CHECK_LE(
			ReprojectionError(pose2, triangulated_point, image_point2),
			max_reprojection_error);
	}

	void TestTriangulationManyPoints(const double projection_noise,
		const double max_reprojection_error) 
	{
		using Eigen::AngleAxisd;

		static const int num_views = 8;

		// Sets some test rotations and translations.
		static const Eigen::Quaterniond kRotations[num_views] = {
			Eigen::Quaterniond(
			AngleAxisd(DegToRad(7.0), Eigen::Vector3d(0.0, 0.0, 1.0).normalized())),
			Eigen::Quaterniond(
			AngleAxisd(DegToRad(12.0), Eigen::Vector3d(0.0, 1.0, 0.0).normalized())),
			Eigen::Quaterniond(
			AngleAxisd(DegToRad(15.0), Eigen::Vector3d(1.0, 0.0, 0.0).normalized())),
			Eigen::Quaterniond(
			AngleAxisd(DegToRad(20.0), Eigen::Vector3d(1.0, 0.0, 1.0).normalized())),
			Eigen::Quaterniond(
			AngleAxisd(DegToRad(11.0), Eigen::Vector3d(0.0, 1.0, 1.0).normalized())),
			Eigen::Quaterniond(
			AngleAxisd(DegToRad(0.0), Eigen::Vector3d(1.0, 1.0, 1.0).normalized())),
			Eigen::Quaterniond(
			AngleAxisd(DegToRad(5.0), Eigen::Vector3d(0.0, 1.0, 1.0).normalized())),
			Eigen::Quaterniond(AngleAxisd(DegToRad(0.0), Eigen::Vector3d(1.0, 1.0, 1.0).normalized()))
		};

		static const Eigen::Vector3d kTranslations[num_views] = {
			Eigen::Vector3d(1.0, 1.0, 1.0),
			Eigen::Vector3d(3.0, 2.0, 13.0),
			Eigen::Vector3d(4.0, 5.0, 11.0),
			Eigen::Vector3d(1.0, 2.0, 15.0),
			Eigen::Vector3d(3.0, 1.5, 91.0),
			Eigen::Vector3d(1.0, 7.0, 11.0),
			Eigen::Vector3d(0.0, 0.0, 0.0),  // Tests no translation.
			Eigen::Vector3d(0.0, 0.0, 0.0)  // Tests no translation and no rotation.
		};

		// Set up model points.
		static const double kTestPoints[][3] = {
			{ -1.62, -2.99, 6.12 }, { 4.42, -1.53, 9.83 }, { 1.45, -0.59, 5.29 },
			{ 1.89, -1.10, 8.22 }, { -0.21, 2.38, 5.63 }, { 0.61, -0.97, 7.49 },
			{ 0.48, 0.70, 8.94 }, { 1.65, -2.56, 8.63 }, { 2.44, -0.20, 7.78 },
			{ 2.84, -2.58, 7.35 }, { -1.35, -2.84, 7.33 }, { -0.42, 1.54, 8.86 },
			{ 2.56, 1.72, 7.86 }, { 1.75, -1.39, 5.73 }, { 2.08, -3.91, 8.37 },
			{ -0.91, 1.36, 9.16 }, { 2.84, 1.54, 8.74 }, { -1.01, 3.02, 8.18 },
			{ -3.73, -0.62, 7.81 }, { -2.98, -1.88, 6.23 }, { 2.39, -0.19, 6.47 },
			{ -0.63, -1.05, 7.11 }, { -1.76, -0.55, 5.18 }, { -3.19, 3.27, 8.18 },
			{ 0.31, -2.77, 7.54 }, { 0.54, -3.77, 9.77 },
		};

		Eigen::Matrix3d calibration;
		calibration <<
			800.0, 0.0, 600.0,
			0.0, 800.0, 400.0,
			0.0, 0.0, 1.0;

		// Set up pose matrices.
		std::vector<Matrix3x4d> poses(num_views);
		for (int i = 0; i < num_views; i++) {
			poses[i] << kRotations[i].toRotationMatrix(), kTranslations[i];
		}

		for (int j = 0; j < ARRAYSIZE(kTestPoints); j++) {
			// Reproject model point into the images.
			std::vector<Eigen::Vector2d> image_points(num_views);
			const Eigen::Vector3d model_point(kTestPoints[j][0], kTestPoints[j][1],
				kTestPoints[j][2]);
			for (int i = 0; i < num_views; i++) {
				image_points[i] =
					(poses[i] * model_point.homogeneous()).eval().hnormalized();
			}

			// Add projection noise if required.
			if (projection_noise) {
				for (int i = 0; i < num_views; i++) {
					test::AddNoiseToProjection(projection_noise, &image_points[i]);
				}
			}

			Eigen::Vector4d triangulated_point;
			CHECK(TriangulateNView(poses, image_points, &triangulated_point));

			// Check the reprojection error.
			for (int i = 0; i < num_views; i++) {
				CHECK_LE(ReprojectionError(poses[i],
					triangulated_point, image_points[i]),
					max_reprojection_error);
			}
		}
	}

	bool TestIsTriangulatedPointInFrontOfCameras(
		const Eigen::Vector3d& point3d,
		const Eigen::Matrix3d& rotation,
		const Eigen::Vector3d& translation,
		const bool expected_outcome)
	{
		FeatureCorrespondence correspondence;
		Eigen::Vector2d point = point3d.hnormalized();
		correspondence.feature1.x = point.x();
		correspondence.feature1.y = point.y();

		point = (rotation * point3d + translation).hnormalized();
		correspondence.feature2.x = point.x();
		correspondence.feature2.y = point.y();
		
		const Eigen::Vector3d position = -rotation.transpose() * translation;
		CHECK_EQ(IsTriangulatedPointInFrontOfCameras(
			correspondence,
			rotation,
			position),
			expected_outcome);

		return true;
	}

}

CTriangulationTest::CTriangulationTest()
{
}


CTriangulationTest::~CTriangulationTest()
{
}

bool CTriangulationTest::Test()
{
	LOG(INFO);

	//CHECK(TriangulateBasicTest());
	//CHECK(TriangulationNoiseTest());
	//
	//CHECK(TriangulationDLTBasicTest());
	//CHECK(TriangulationDLTNoiseTest());

	CHECK(TriangulationMidpointNoiseTest());
	CHECK(TriangulationMidpointBasicTest());

	CHECK(TriangulationNViewBasicTest());
	CHECK(TriangulationNViewNoiseTest());

	CHECK(IsTriangulatedPointInFrontOfCamerasInFront());
	CHECK(IsTriangulatedPointInFrontOfCamerasBehind());
	CHECK(IsTriangulatedPointInFrontOfCamerasOneInFrontOneBehind());

	CHECK(SufficientTriangulationAngleAllSufficient());
	CHECK(SufficientTriangulationAngleAllInsufficient());
	CHECK(SufficientTriangulationAngleSomeInsufficient());
	CHECK(SufficientTriangulationAngleTwoInsufficient());

	LOG(INFO);

	return true;
}

bool CTriangulationTest::TriangulateBasicTest()
{
	static const double kProjectionNoise = 0.0;
	static const double kReprojectionTolerance = 1e-12;

	// Set up model points.
	const Eigen::Vector3d points_3d[2] = { Eigen::Vector3d(5.0, 20.0, 23.0),
		Eigen::Vector3d(-6.0, 16.0, 33.0) };

	// Set up rotations.
	const Eigen::Quaterniond kRotation(Eigen::AngleAxisd(0.15, Eigen::Vector3d(0.0, 1.0, 0.0)));

	// Set up translations.
	const Eigen::Vector3d kTranslation(-3.0, 1.5, 11.0);

	// Run the test.
	for (int i = 0; i < 2; i++) {
		TestTriangulationBasic(TriangulationType::STANDARD,
			points_3d[i],
			kRotation,
			kTranslation,
			kProjectionNoise,
			kReprojectionTolerance);
	}

	return true;
}

bool CTriangulationTest::TriangulationNoiseTest()
{
	static const double kProjectionNoise = 1.0 / 512.0;
	static const double kReprojectionTolerance = 1e-5;

	// Set up model points.
	const Eigen::Vector3d points_3d[2] = { Eigen::Vector3d(5.0, 20.0, 23.0),
		Eigen::Vector3d(-6.0, 16.0, 33.0) };

	// Set up rotations.
	const Eigen::Quaterniond kRotation(Eigen::AngleAxisd(0.15, Eigen::Vector3d(0.0, 1.0, 0.0)));

	// Set up translations.
	const Eigen::Vector3d kTranslation(-3.0, 1.5, 11.0);

	// Run the test.
	for (int i = 0; i < 2; i++) {
		TestTriangulationBasic(TriangulationType::STANDARD,
			points_3d[i],
			kRotation,
			kTranslation,
			kProjectionNoise,
			kReprojectionTolerance);
	}

	return true;
}

bool CTriangulationTest::TriangulationDLTBasicTest()
{
	static const double kProjectionNoise = 0.0;
	static const double kReprojectionTolerance = 1e-12;

	// Set up model points.
	const Eigen::Vector3d points_3d[2] = { Eigen::Vector3d(5.0, 20.0, 23.0),
		Eigen::Vector3d(-6.0, 16.0, 33.0) };

	// Set up rotations.
	const Eigen::Quaterniond kRotation(Eigen::AngleAxisd(0.15, Eigen::Vector3d(0.0, 1.0, 0.0)));

	// Set up translations.
	const Eigen::Vector3d kTranslation(-3.0, 1.5, 11.0);

	// Run the test.
	for (int i = 0; i < 2; i++) {
		TestTriangulationBasic(TriangulationType::DLT,
			points_3d[i],
			kRotation,
			kTranslation,
			kProjectionNoise,
			kReprojectionTolerance);
	}
	return true;
}

bool CTriangulationTest::TriangulationDLTNoiseTest()
{
	static const double kProjectionNoise = 1.0 / 512.0;
	static const double kReprojectionTolerance = 1e-5;

	// Set up model points.
	const Eigen::Vector3d points_3d[2] = { Eigen::Vector3d(5.0, 20.0, 23.0),
		Eigen::Vector3d(-6.0, 16.0, 33.0) };

	// Set up rotations.
	const Eigen::Quaterniond kRotation(Eigen::AngleAxisd(0.15, Eigen::Vector3d(0.0, 1.0, 0.0)));

	// Set up translations.
	const Eigen::Vector3d kTranslation(-3.0, 1.5, 11.0);

	// Run the test.
	for (int i = 0; i < 2; i++) {
		TestTriangulationBasic(TriangulationType::DLT,
			points_3d[i],
			kRotation,
			kTranslation,
			kProjectionNoise,
			kReprojectionTolerance);
	}
	return true;
}

bool CTriangulationTest::TriangulationMidpointBasicTest()
{
	static const double kProjectionNoise = 0.0;
	static const double kReprojectionTolerance = 1e-12;

	// Set up model points.
	const Eigen::Vector3d points_3d[2] = { Eigen::Vector3d(5.0, 20.0, 23.0),
		Eigen::Vector3d(-6.0, 16.0, 33.0) };

	// Set up rotations.
	const Eigen::Quaterniond kRotation(Eigen::AngleAxisd(0.15, Eigen::Vector3d(0.0, 1.0, 0.0)));

	// Set up translations.
	const Eigen::Vector3d kTranslation(-3.0, 1.5, 11.0);

	// Run the test.
	for (int i = 0; i < 2; i++) {
		TestTriangulationBasic(TriangulationType::MIDPOINT,
			points_3d[i],
			kRotation,
			kTranslation,
			kProjectionNoise,
			kReprojectionTolerance);
	}

	return true;
}

bool CTriangulationTest::TriangulationMidpointNoiseTest()
{
	static const double kProjectionNoise = 1.0 / 512.0;
	static const double kReprojectionTolerance = 1e-5;

	// Set up model points.
	const Eigen::Vector3d points_3d[2] = { Eigen::Vector3d(5.0, 20.0, 23.0),
		Eigen::Vector3d(-6.0, 16.0, 33.0) };

	// Set up rotations.
	const Eigen::Quaterniond kRotation(Eigen::AngleAxisd(0.15, Eigen::Vector3d(0.0, 1.0, 0.0)));

	// Set up translations.
	const Eigen::Vector3d kTranslation(-3.0, 1.5, 11.0);

	// Run the test.
	for (int i = 0; i < 2; i++) {
		TestTriangulationBasic(TriangulationType::MIDPOINT,
			points_3d[i],
			kRotation,
			kTranslation,
			kProjectionNoise,
			kReprojectionTolerance);
	}

	return true;
}

bool CTriangulationTest::TriangulationNViewBasicTest()
{
	static const double kProjectionNoise = 0.0;
	static const double kReprojectionTolerance = 1e-12;

	// Run the test.
	TestTriangulationManyPoints(kProjectionNoise, kReprojectionTolerance);

	return true;
}

bool CTriangulationTest::TriangulationNViewNoiseTest()
{
	static const double kProjectionNoise = 1.0 / 512.0;
	static const double kReprojectionTolerance = 5e-4;

	// Run the test.
	TestTriangulationManyPoints(kProjectionNoise, kReprojectionTolerance);

	return true;
}

bool CTriangulationTest::IsTriangulatedPointInFrontOfCamerasInFront()
{
	const Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
	const Eigen::Vector3d position(-1, 0, 0);
	const Eigen::Vector3d point(0, 0, 5);
	TestIsTriangulatedPointInFrontOfCameras(point, rotation, position, true);

	return true;
}

bool CTriangulationTest::IsTriangulatedPointInFrontOfCamerasBehind()
{
	const Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
	const Eigen::Vector3d position(-1, 0, 0);
	const Eigen::Vector3d point(0, 0, -5);
	TestIsTriangulatedPointInFrontOfCameras(point, rotation, position, false);

	return true;
}

bool CTriangulationTest::IsTriangulatedPointInFrontOfCamerasOneInFrontOneBehind()
{
	const Eigen::Matrix3d rotation = Eigen::Matrix3d::Identity();
	const Eigen::Vector3d position(0, 0, -2);
	const Eigen::Vector3d point(0, 0, 1);
	TestIsTriangulatedPointInFrontOfCameras(point, rotation, position, false);

	return true;
}

bool CTriangulationTest::SufficientTriangulationAngleAllSufficient()
{
	static const double kMinSufficientAngle = 4.0;
	static const double kAngleBetweenCameras = 5.0;

	// Try varying numbers of cameras observing a 3d point from 2 to 50 cameras.
	for (int i = 2; i < 50; i++) {
		// Set up cameras on a unit circle so that the angles are known. Assume that
		// the triangulated point is at (0, 0, 0).
		std::vector<Eigen::Vector3d> rays;
		for (int j = 0; j < i; j++) {
			rays.emplace_back(cos(DegToRad(j * kAngleBetweenCameras)),
				sin(DegToRad(j * kAngleBetweenCameras)),
				0.0);
		}

		CHECK(SufficientTriangulationAngle(rays, kMinSufficientAngle));
	}

	return true;
}

bool CTriangulationTest::SufficientTriangulationAngleAllInsufficient()
{
	static const double kMinSufficientAngle = 4.0;

	// Try varying numbers of cameras observing a 3d point from 2 to 50 cameras.
	for (int i = 2; i < 50; i++) {
		// Set up cameras on a unit circle so that the angles are known. Assume that
		// the triangulated point is at (0, 0, 0).
		std::vector<Eigen::Vector3d> rays;
		const double angle = kMinSufficientAngle / static_cast<double>(i + 1e-4);
		for (int j = 0; j < i; j++) {
			rays.emplace_back(cos(DegToRad(j * angle)),
				sin(DegToRad(j * angle)),
				0.0);
		}

		CHECK(!SufficientTriangulationAngle(rays, kMinSufficientAngle));
	}
	return true;
}

bool CTriangulationTest::SufficientTriangulationAngleSomeInsufficient()
{
	static const double kMinSufficientAngle = 4.0;

	// Set up cameras on a unit circle so that the angles are known. Assume that
	// the triangulated point is at (0, 0, 0).
	std::vector<Eigen::Vector3d> rays;
	rays.emplace_back(cos(DegToRad(0)), sin(DegToRad(0)), 0.0);
	rays.emplace_back(cos(DegToRad(5.0)), sin(DegToRad(5.0)), 0.0);
	rays.emplace_back(cos(DegToRad(1.0)), sin(DegToRad(1.0)), 0.0);

	// We only need one pair of rays to have a sufficient viewing angle, so this
	// should return true since views 0 and 2 have a viewing angle of 5 degree.
	CHECK(SufficientTriangulationAngle(rays, kMinSufficientAngle));
	
	return true;
}

bool CTriangulationTest::SufficientTriangulationAngleTwoInsufficient()
{
	static const double kMinSufficientAngle = 4.0;

	// Set up cameras on a unit circle so that the angles are known. Assume that
	// the triangulated point is at (0, 0, 0).
	std::vector<Eigen::Vector3d> rays;
	rays.emplace_back(cos(DegToRad(0)), sin(DegToRad(0)), 0.0);
	rays.emplace_back(cos(DegToRad(1.0)), sin(DegToRad(1.0)), 0.0);

	// We only need one pair of rays to have a sufficient viewing angle, so this
	// should return true since views 0 and 2 have a viewing angle of 5 degree.
	CHECK(!SufficientTriangulationAngle(rays, kMinSufficientAngle));

	return true;
}
