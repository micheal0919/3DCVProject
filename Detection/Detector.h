#ifndef __DETECTION_H__
#define __DETECTION_H__

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "DirectShowVideoGrabber.h"

class CDetector
{
public:
	struct Options
	{
		std::string yml_read_path;
	};

public:
	CDetector(const Options& options);
	~CDetector();

	bool Detect();

private:
	bool ReadYmlFile();

private:
	Options m_options;
	std::vector<cv::Point3d> m_3d_points;
	cv::Mat m_descriotors;
	DirectShowVideoGrabber m_cap;
};

#endif // __DETECTION_H__
