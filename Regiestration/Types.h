#ifndef __TYPES_H__
#define __TYPES_H__

#include <stdint.h>
#include <cstdint>
#include <limits>
#include <utility>

#include <Eigen/Core>
#include <opencv2/opencv.hpp>

// typedef Eigen::Vector2d Feature;
// It's not compitable with std in windows
typedef cv::Point2d Feature;

typedef uint32_t ViewId;
typedef uint32_t TrackId;
typedef std::pair<ViewId, ViewId> ViewIdPair;

static const ViewId kInvalidViewId = std::numeric_limits<ViewId>::max();
static const ViewId kInvalidTrackId = std::numeric_limits<TrackId>::max();

// Used as the projection matrix type.
typedef Eigen::Matrix<double, 3, 4> Matrix3x4d;

struct CamaraProjection
{
	cv::Mat mat;
};

struct VerifyTwoViewMatchesOptions
{

};

// The feature location of two correspondences. These can be pixel coordinates
// or normalized coordinates.
struct FeatureCorrespondence {
public:
	Feature feature1;
	Feature feature2;
	bool operator==(const FeatureCorrespondence& other) const 
	{
		return (feature1 == other.feature1 && feature2 == other.feature1);
	}

};

// A struct to hold match and projection data between two views. It is assumed
// that the first view is at the origin with an identity rotation.
struct TwoViewInfo {
public:
	TwoViewInfo()
		: focal_length_1(0.0),
		focal_length_2(0.0),
		position_2(Eigen::Vector3d::Zero()),
		rotation_2(Eigen::Vector3d::Zero()),
		num_verified_matches(0),
		num_homography_inliers(0) {}

	double focal_length_1;
	double focal_length_2;

	Eigen::Vector3d position_2;
	Eigen::Vector3d rotation_2;

	// Number of features that were matched and geometrically verified betwen the
	// images.
	int num_verified_matches;

	// Number of inliers based on homography estimation. This is useful for
	// incremental SfM for choosing an initial view pair for the reconstruction.
	int num_homography_inliers;
};

struct ImagePairMatch {
public:
	std::string image1;
	std::string image2;

	// TODO:
	// If the matches are verified matches then the two view info contains the
	// relative pose information between the images.
	TwoViewInfo twoview_info;

	// Feature locations in pixel coordinates. If the match is a verified match
	// then this only contains inlier correspondences.
	std::vector<FeatureCorrespondence> correspondences;
};

struct BundleAdjustmentOptions
{
	bool option = false;
};

// Weak calibration is not always available, so we need this helper struct to
// keep track of which data fields have been set.
struct Prior {
	bool is_set = false;
	double value = 0;
};

// Prior information about a View. This is typically gathered from EXIF or
// sensor data that provides weak calibration.
struct CameraIntrinsicsPrior {
	// The image size *should* always be set, so we don't have to worry about
	// making it an Prior type.
	int image_width;
	int image_height;

	Prior focal_length;
	Prior principal_point[2];
	Prior aspect_ratio;
	Prior skew;
	Prior radial_distortion[2];

};

//struct ImageInfo
//{
//	std::string image_name;
//	int image_width;
//	int image_height;
//
//	cv::Mat K_mat;
//	cv::Mat RT_mat;
//};

#endif // __TYPES_H__
