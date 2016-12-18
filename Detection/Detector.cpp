#include "Detector.h"

#include <glog/logging.h>
#include <opencv2/opencv.hpp>



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
		LOG(ERROR) << "Fail to open video capture" << std::endl;
		return false;
	}
	LOG(INFO);

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



}

bool CDetector::Detect()
{
	// check if we succeeded
	if (!m_cap.init())
	{
		LOG(ERROR) << "Fail to open video capture" << std::endl;
		return false;
	}

	while (true)
	{
		IplImage * live_video = m_cap.getImage();
		LOG(INFO) << "Image size " << live_video->width << "x" << live_video->height << ", " << live_video->nChannels << " channels/pixel" << ", " << live_video->depth << " bits/channel" << std::endl;
		cvShowImage("MarkerTrackerTest", live_video);
	}

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
	LOG(INFO);

	return true;
}