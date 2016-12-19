#ifndef __DETECTION_H__
#define __DETECTION_H__

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "DirectShowVideoGrabber.h"
#include "RobustMatcher.h"

class CDetector
{
public:
	struct Options
	{
		std::string yml_read_path;
		std::string camera_instrinsics_file_path;
	};

public:
	CDetector(const Options& options);
	~CDetector();

	bool Init();
	bool Detect();

private:
	bool ReadYmlFile();
	bool GetCameraIntrinscis();
	void draw2DPoints(cv::Mat image, std::vector<cv::Point2d> &list_points, cv::Scalar color);


private:
	Options m_options;
	std::vector<cv::Point3d> m_3d_points;
	cv::Mat m_descriotors;
	DirectShowVideoGrabber m_cap;
	cv::Mat m_K_mat;
	CRobustMatcher m_matcher;
};

#endif // __DETECTION_H__
