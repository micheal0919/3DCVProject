#include "TriangulationTest.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glog/logging.h>

#include "Triangulation.h"
#include "TestUitls.h"

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
}

CTriangulationTest::CTriangulationTest()
{
}


CTriangulationTest::~CTriangulationTest()
{
}

bool CTriangulationTest::Test()
{
	return true;
}

bool CTriangulationTest::BasicTest()
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
