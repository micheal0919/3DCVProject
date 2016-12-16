#ifndef __UTIL_H__
#define __UTIL_H__

#include <string>
#include <vector>

#include <Eigen/Core>

#include "Types.h"

class CReconstruction;
struct ImageInfo;

double RadToDeg(double angle_radians);

double DegToRad(double angle_degrees);

std::string GetImageNameFromNum(int num);

// Inverts the two view info such that the focal lengths are swapped and the
// rotation and position are inverted.
void SwapCameras(TwoViewInfo* twoview_info);

// A macro to disallow the copy constructor and operator= functions
// This should be used in the private: declarations for a class
#define DISALLOW_COPY_AND_ASSIGN(TypeName) \
	TypeName(const TypeName&);               \
	void operator=(const TypeName&)

// Determines the array size an array a.
#define ARRAYSIZE(a) \
	((sizeof(a) / sizeof(*(a))) / \
	static_cast<size_t>(!(sizeof(a) % sizeof(*(a)))))

bool ImageInfoToCameraInfo(const ImageInfo& info, CameraIntrinsicsPrior& intrinsics,
	Eigen::Matrix3d& camera_orientation_matrix, Eigen::Vector3d& camera_postion);

bool WriteReconstruction(const CReconstruction& reconstruction,
	const std::string& output_file);

bool ReadReconstruction(const std::string& input_file, std::vector<cv::Point3d>& points_3d);

#endif // __UTIL_H__
