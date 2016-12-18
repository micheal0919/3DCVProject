#include "uitl.h"

#include <opencv2/opencv.hpp>
#include <glog/logging.h>
#include <tinyxml.h>

#include "rotation.h"
#include "Track.h"
#include "View.h"
#include "Reconstruction.h"
#include "ImageInfoIO.h"

const double m_pi = 3.14159265358979323846264338327950288;
static const double kRadToDeg = 180.0 / m_pi;
static const double kDegToRad = m_pi / 180.0;

double RadToDeg(double angle_radians)
{
	return angle_radians * kRadToDeg;
}

double DegToRad(double angle_degrees)
{
	return angle_degrees * kDegToRad;
}

std::string GetImageNameFromNum(int num)
{
	char buf[32];
	sprintf(buf, "%06d.jpg", num);
	std::string name = buf;

	return name;
}

// Inverts the two view info such that the focal lengths are swapped and the
// rotation and position are inverted.
void SwapCameras(TwoViewInfo* twoview_info)
{
	CHECK_NE(twoview_info->focal_length_1, 0.0);
	CHECK_NE(twoview_info->focal_length_2, 0.0);

	// Swap the focal lengths.
	std::swap(twoview_info->focal_length_1, twoview_info->focal_length_2);

	// Invert the rotation.
	twoview_info->rotation_2 *= -1.0;

	// Invert the translation.
	Eigen::Matrix3d rotation_mat;
	ceres::AngleAxisToRotationMatrix(
		twoview_info->rotation_2.data(),
		ceres::ColumnMajorAdapter3x3(rotation_mat.data()));
	twoview_info->position_2 = -rotation_mat * twoview_info->position_2;
}

bool ImageInfoToCameraInfo(const ImageInfo& info, CameraIntrinsicsPrior& intrinsics,
	Eigen::Matrix3d& camera_orientation_matrix, Eigen::Vector3d& camera_postion)
{
	intrinsics.image_width = info.image_width;
	intrinsics.image_height = info.image_height;

	intrinsics.focal_length.value = info.K_mat.at<double>(0, 0);
	intrinsics.focal_length.is_set = true;

	intrinsics.aspect_ratio.value = info.K_mat.at<double>(1, 1) / info.K_mat.at<double>(0, 0);
	intrinsics.aspect_ratio.is_set = true;

	intrinsics.skew.value = info.K_mat.at<double>(0, 1);
	intrinsics.skew.is_set = true;

	intrinsics.principal_point[0].value = info.K_mat.at<double>(0, 2);
	intrinsics.principal_point[0].is_set = true;
	intrinsics.principal_point[1].value = info.K_mat.at<double>(1, 2);
	intrinsics.principal_point[1].is_set = true;

	for (int i = 0; i < 3; i++)
	{
		for (int j = 0; j < 3; j++)
		{
			camera_orientation_matrix(i, j) = info.RT_mat.at<double>(i, j);
		}
	}

	camera_postion(0) = info.RT_mat.at<double>(0, 3);
	camera_postion(1) = info.RT_mat.at<double>(1, 3);
	camera_postion(2) = info.RT_mat.at<double>(2, 3);

	return true;
}

bool ReadReconstruction(const std::string& input_file, std::vector<cv::Point3d>& points_3d)
{
	cv::Mat points3d_mat;

	cv::FileStorage storage(input_file, cv::FileStorage::READ);
	CHECK(storage.isOpened()) << "Fail to open file";

	storage["points_3d"] >> points3d_mat;
	storage.release();

	points3d_mat.copyTo(points_3d);

	return true;
}
