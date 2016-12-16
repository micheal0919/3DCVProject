#ifndef __IMAGE_CAPTURE_H__
#define __IMAGE_CAPTURE_H__

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>

#include "MarkerTrackerInterface.h"
#include "DirectShowVideoGrabber.h"

class CPoseEstimator
{
public:
	struct Options 
	{
		std::string camera_intrin_file_path;
		std::string marker_world_file_path;
		std::string image_path;
		std::string image_info_file_path;
		int num_image;
	};
	

	CPoseEstimator(const Options& option);
	~CPoseEstimator();

	bool Init();

	bool EstimatePoses();

private:
	Options m_options;
//	cv::VideoCapture m_cap;
	DirectShowVideoGrabber m_cap;
	MarkerTrackerInterface m_marker_tracker;
};

#endif // __IMAGE_CAPTURE_H__
