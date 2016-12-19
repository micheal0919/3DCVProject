#include "Detector.h"

#include <vector>

#include <glog/logging.h>
#include <opencv2/opencv.hpp>

namespace
{
	// For circles
	int lineType = 8;
	int radius = 4;
	double thickness_circ = -1;
}

CDetector::CDetector(const CDetector::Options& options)
:m_options(options)
{
}


CDetector::~CDetector()
{
}

bool CDetector::Init()
{
	LOG(INFO) << "Beginning of CDetector::Init";

	// check if we can init the camera
	if (!m_cap.init())
	{
		LOG(ERROR) << "Fail to open video capture";
		return false;
	}
	LOG(INFO);

	if (!ReadYmlFile())
	{
		LOG(ERROR) << "Fail to read point info file";
		return false;
	}

	if (!GetCameraIntrinscis())
	{
		LOG(ERROR) << "Fail to get camera intrinsics";
		return false;
	}

	LOG(INFO) << "Endding of CDetector::Init";

	return true;
}

bool CDetector::Detect()
{
	LOG(INFO) << "Beginnig of CDetector::Detect";

	cv::namedWindow("imgshow", cv::WINDOW_AUTOSIZE);
	while (true)
	{
		IplImage * live_video = m_cap.getImage();
		LOG(INFO) << "Get the live video form camera";

		cv::Mat live = cv::Mat(live_video);
		cv::Mat live_show = live.clone();

		// -- Step 1: Robust matching between model descriptors and scene descriptors

		std::vector<cv::DMatch> good_matches;       // to obtain the 3D points of the model
		std::vector<cv::KeyPoint> keypoints_scene;  // to obtain the 2D points of the scene

		m_matcher.RobustMatch(live, good_matches, keypoints_scene, m_descriotors);

		// -- Step 2: Find out the 2D/3D correspondences

		cv::vector<cv::Point3d> list_points3d_model_match; // container for the model 3D coordinates found in the scene
		cv::vector<cv::Point2d> list_points2d_scene_match; // container for the model 2D coordinates found in the scene

		for (unsigned int match_index = 0; match_index < good_matches.size(); ++match_index)
		{
			cv::Point3d point3d_model = m_3d_points[good_matches[match_index].trainIdx];  // 3D point from model
			cv::Point2d point2d_scene = keypoints_scene[good_matches[match_index].queryIdx].pt; // 2D point from the scene
			list_points3d_model_match.push_back(point3d_model);         // add 3D point
			list_points2d_scene_match.push_back(point2d_scene);         // add 2D point
		}

		// Draw outliers
		Draw2DPoints(live_show, list_points2d_scene_match, cv::Scalar(0, 0, 255));


		cv::Mat inliers_idx;
		std::vector<cv::Point2d> list_points2d_inliers;
		if (good_matches.size() > 0) // None matches, then RANSAC crashes
		{

			// -- Step 3: Estimate the pose using RANSAC approach
			cv::Mat distCoeffs = cv::Mat::zeros(4, 1, CV_64FC1);  // vector of distortion coefficients
			cv::Mat rvec = cv::Mat::zeros(3, 1, CV_64FC1);          // output rotation vector
			cv::Mat tvec = cv::Mat::zeros(3, 1, CV_64FC1);    // output translation vector
			bool useExtrinsicGuess = false;   // if true the function uses the provided rvec and tvec values as
					// initial approximations of the rotation and translation vectors
			cv::solvePnPRansac(list_points3d_model_match, list_points2d_scene_match, m_K_mat, distCoeffs, rvec, tvec,
				useExtrinsicGuess);

			LOG(INFO) << "The rotation vetor of camera pose is " << rvec;
			LOG(INFO) << "The translation vector of camera pose is " << tvec;
			DrawCoordinate(live_show, rvec, tvec);
			LOG(INFO);
		}
		LOG(INFO);

		cv::imshow("imgshow", live_show);
		LOG(INFO);
	}

	LOG(INFO) << "Beginnig of CDetector::Detect";

	return true;
}

bool CDetector::ReadYmlFile()
{
	LOG(INFO) << "Beginning of CDetector::ReadYmlFile";

	cv::FileStorage fs(m_options.yml_read_path, cv::FileStorage::READ);
	CHECK(fs.isOpened()) << "Fail to open reconstruction file";

	cv::Mat point_mat;
	fs["points_3d"] >> point_mat;
	point_mat.copyTo(m_3d_points);

	fs["descriptors"] >> m_descriotors;
	fs.release();

	LOG(INFO) << "The num of 3d points is " << m_3d_points.size();
	LOG(INFO) << "The num of descriptors is " << m_descriotors.rows;

	CHECK_EQ(m_3d_points.size(), m_descriotors.rows) << "The num of 3d points is NOT equal to the num of descriptors";

	LOG(INFO) << "Endding of CDetector::ReadYmlFile";

	return true;
}

bool CDetector::GetCameraIntrinscis()
{
	LOG(INFO) << "Beginning of CDetector::GetCameraIntrinscis";

	m_K_mat = cv::Mat::zeros(3, 3, CV_64FC1);
	m_K_mat.at<double>(0, 0) = 621.900800;       //      [ fx   0  cx ]
	m_K_mat.at<double>(1, 1) = 326.661120;       //      [  0  fy  cy ]
	m_K_mat.at<double>(0, 2) = 467.319600;       //      [  0   0   1 ]
	m_K_mat.at<double>(1, 2) = 176.396760;
	m_K_mat.at<double>(2, 2) = 1;

	LOG(INFO) << "Endding of CDetector::GetCameraIntrinscis";

	return true;
}

// Draw only the 2D points
void CDetector::Draw2DPoints(cv::Mat& image, std::vector<cv::Point2d> &list_points, cv::Scalar color)
{
	for (size_t i = 0; i < list_points.size(); i++)
	{
		cv::Point2d point_2d = list_points[i];

		// Draw Selected points
		cv::circle(image, point_2d, radius, color, -1, lineType);
	}
}

void CDetector::DrawCoordinate(cv::Mat& image, const cv::Mat& rvec, const cv::Mat& tvec)
{
	LOG(INFO);
	const double scale = 30.0;

	cv::Mat point0(3, 1, CV_64FC1);
	point0.at<double>(0, 0) = 0;
	point0.at<double>(1, 0) = 0;
	point0.at<double>(2, 0) = 0;

	cv::Mat pointX(3, 1, CV_64FC1);
	pointX.at<double>(0, 0) = scale;
	pointX.at<double>(1, 0) = 0;
	pointX.at<double>(2, 0) = 0;

	cv::Mat pointY(3, 1, CV_64FC1);
	pointY.at<double>(0, 0) = 0;
	pointY.at<double>(1, 0) = scale;
	pointY.at<double>(2, 0) = 0;

	cv::Mat pointZ(3, 1, CV_64FC1);
	pointZ.at<double>(0, 0) = 0;
	pointZ.at<double>(1, 0) = 0;
	pointZ.at<double>(2, 0) = scale;


	cv::Mat rmat(3, 3, CV_64FC1);
	cv::Rodrigues(rvec, rmat);
	LOG(INFO) << "rvec = " << rvec;
	LOG(INFO) << "rmat = " << rmat;

	cv::Mat projected0(3, 1, CV_64FC1);
	projected0 = rmat * point0;
	projected0 = projected0 + tvec;
	projected0 = m_K_mat * projected0;
	LOG(INFO) << "projected0 = " << projected0;
	double tmp_scale = projected0.at<double>(2, 0);
	if (0 == tmp_scale)
	{
		tmp_scale = 1;
	}
	projected0 = projected0 / tmp_scale;
	LOG(INFO);

	cv::Mat projectedX(3, 1, CV_64FC1);
	projectedX = rmat * pointX;
	projectedX = projectedX + tvec;
	projectedX = m_K_mat * projectedX;
	tmp_scale = projectedX.at<double>(2, 0);
	if (0 == tmp_scale)
	{
		tmp_scale = 1;
	}
	projectedX = projectedX / tmp_scale;

	cv::Mat projectedY(3, 1, CV_64FC1);
	projectedY = rmat * pointY;
	projectedY = projectedY + tvec;
	projectedY = m_K_mat * projectedY;
	tmp_scale = projectedY.at<double>(2, 0);
	if (0 == tmp_scale)
	{
		tmp_scale = 1;
	}
	projectedY = projectedY / tmp_scale;

	cv::Mat projectedZ(3, 1, CV_64FC1);
	projectedZ = rmat * pointZ;
	projectedZ = projectedZ + tvec;
	projectedZ = m_K_mat * projectedZ;
	tmp_scale = projectedZ.at<double>(2, 0);
	if (0 == tmp_scale)
	{
		tmp_scale = 1;
	}
	projectedZ = projectedZ / tmp_scale;

	cv::Point2d point_0;
	point_0.x = projected0.at<double>(0, 0);
	point_0.y = projected0.at<double>(1, 0);

	cv::Point2d point_x;
	point_x.x = projectedX.at<double>(0, 0);
	point_x.y = projectedX.at<double>(1, 0);
	
	cv::Point2d point_y;
	point_y.x = projectedY.at<double>(0, 0);
	point_y.y = projectedY.at<double>(1, 0);

	cv::Point2d point_z;
	point_z.x = projectedZ.at<double>(0, 0);
	point_z.y = projectedZ.at<double>(1, 0);

	std::vector<cv::Point2d> list_points2d;
	list_points2d.emplace_back(point_0);
	list_points2d.emplace_back(point_x);
	list_points2d.emplace_back(point_y);
	list_points2d.emplace_back(point_z);

	LOG(INFO);

	Draw3DCoordinateAxes(image, list_points2d);
	
	LOG(INFO);
}

// Draw an arrow into the image
void CDetector::DrawArrow(cv::Mat image, cv::Point2d p, cv::Point2d q, cv::Scalar color, int arrowMagnitude, int thickness, int line_type, int shift)
{
	LOG(INFO);

	//Draw the principle line
	cv::line(image, p, q, color, thickness, line_type, shift);
	const double PI = CV_PI;
	//compute the angle alpha
	double angle = atan2((double)p.y - q.y, (double)p.x - q.x);
	//compute the coordinates of the first segment
	p.x = (int)(q.x + arrowMagnitude * cos(angle + PI / 4));
	p.y = (int)(q.y + arrowMagnitude * sin(angle + PI / 4));
	//Draw the first segment
	cv::line(image, p, q, color, thickness, line_type, shift);
	//compute the coordinates of the second segment
	p.x = (int)(q.x + arrowMagnitude * cos(angle - PI / 4));
	p.y = (int)(q.y + arrowMagnitude * sin(angle - PI / 4));
	//Draw the second segment
	cv::line(image, p, q, color, thickness, line_type, shift);
	LOG(INFO);
}

// Draw the 3D coordinate axes
void CDetector::Draw3DCoordinateAxes(cv::Mat image, const std::vector<cv::Point2d> &list_points2d)
{
	LOG(INFO);

	cv::Scalar red(0, 0, 255);
	cv::Scalar green(0, 255, 0);
	cv::Scalar blue(255, 0, 0);
	cv::Scalar black(0, 0, 0);

	cv::Point2i origin = list_points2d[0];
	cv::Point2i pointX = list_points2d[1];
	cv::Point2i pointY = list_points2d[2];
	cv::Point2i pointZ = list_points2d[3];

	DrawArrow(image, origin, pointX, red, 9, 2);
	DrawArrow(image, origin, pointY, blue, 9, 2);
	DrawArrow(image, origin, pointZ, green, 9, 2);
	cv::circle(image, origin, radius / 2, black, -1, lineType);
	LOG(INFO);

}
