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

	// Draw only the 2D points
	void Draw2DPoints(cv::Mat& image, std::vector<cv::Point2d> &list_points, cv::Scalar color);
	
	// Draw a coordinate into the image
	void DrawCoordinate(cv::Mat& image, const cv::Mat& rvec, const cv::Mat& tvec);
	
	// Draw an arrow into the image
	void DrawArrow(cv::Mat image, cv::Point2d p, cv::Point2d q, cv::Scalar color, int arrowMagnitude = 9, int thickness = 1, int line_type = 8, int shift = 0);

	// Draw the 3D coordinate axes
	void Draw3DCoordinateAxes(cv::Mat image, const std::vector<cv::Point2d> &list_points2d);

private:
	Options m_options;
	std::vector<cv::Point3d> m_3d_points;
	cv::Mat m_descriotors;
	DirectShowVideoGrabber m_cap;
	cv::Mat m_K_mat;
	CRobustMatcher m_matcher;
};

#endif // __DETECTION_H__
