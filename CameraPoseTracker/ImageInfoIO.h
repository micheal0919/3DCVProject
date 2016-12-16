#ifndef __IMAGE_INFO_IO_H__
#define __IMAGE_INFO_IO_H__

#include <string>
#include <vector>
#include <opencv2/opencv.hpp>

struct ImageInfo
{
	std::string image_name;
	int image_width;
	int image_height;

	cv::Mat K_mat;
	cv::Mat RT_mat;
};

bool WriteImageInfoToFile(const std::string& file_path, const std::vector<ImageInfo>& infos);

bool ReadImageInfoFromFile(const std::string& file_path, std::vector<ImageInfo>& infos);

#endif // __IMAGE_INFO_IO_H__
