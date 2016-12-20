#ifndef __ROBUSTMATCHER_H__
#define __ROBUSTMATCHER_H__

#include <opencv2/opencv.hpp>
#include "Types.h"

#include <glog/logging.h>

class CRobustMatcher
{
public:
	CRobustMatcher() : m_ratio(0.8f)
	{
		// ORB is the default feature
//		m_detector = cv::FeatureDetector::create("SIFT");
		m_detector = cv::makePtr<cv::ORB>(1000);
		m_extractor = m_detector;

		// instantiate LSH index parameters
		cv::Ptr<cv::flann::IndexParams> indexParams = cv::makePtr<cv::flann::LshIndexParams>(6, 12, 1);
		// instantiate flann search parameters
		cv::Ptr<cv::flann::SearchParams> searchParams = cv::makePtr<cv::flann::SearchParams>(50);
		LOG(INFO) << std::endl;

		// instantiate FlannBased matcher
		m_matcher = cv::makePtr<cv::FlannBasedMatcher>(indexParams, searchParams);
		LOG(INFO) << std::endl;
	}

	~CRobustMatcher() {}

	// Set the feature detector
//	void SetFeatureDetector(const cv::Ptr<cv::FeatureDetector>& detect) { m_detector = detect; }

	// Set the descriptor extractor
//	void SetDescriptorExtractor(const cv::Ptr<cv::DescriptorExtractor>& desc) { m_extractor = desc; }

	// Set ratio parameter for the ratio test
//	void SetRatio(float rat) { m_ratio = rat; }

	// Set the matcher
//	void SetDescriptorMatcher(const cv::Ptr<cv::DescriptorMatcher>& match) { m_matcher = match; }

	// Match feature points using ratio and symmetry test
	void RobustMatch(const std::string& image_path_1, const std::string& image_path_2, ImagePairMatch& match);

	// Match feature points using ratio test
	void FastRobustMatch(const std::string& image_path_1, const std::string& image_path_2, ImagePairMatch& match);

	// Match feature points using ratio and symmetry test
	void RobustMatch(const cv::Mat& frame, std::vector<cv::DMatch>& good_matches,
		std::vector<cv::KeyPoint>& keypoints_frame,
		const cv::Mat& descriptors_model);

	// Match feature points using ratio test
	void FastRobustMatch(const cv::Mat& frame, std::vector<cv::DMatch>& good_matches,
		std::vector<cv::KeyPoint>& keypoints_frame,
		const cv::Mat& descriptors_model);
	void ComputeKeyPoints(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints);
	void ComputeDescriptors(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors);

private:

	int RatioTest(std::vector<std::vector<cv::DMatch> > &matches);

	// Insert symmetrical matches in symMatches vector
	void SymmetryTest(const std::vector<std::vector<cv::DMatch> >& matches1,
		const std::vector<std::vector<cv::DMatch> >& matches2,
		std::vector<cv::DMatch>& symMatches);

	void HomographyTest(const std::vector<cv::KeyPoint>& keypoints_1, const std::vector<cv::KeyPoint>& keypoints_2, 
		const std::vector<cv::DMatch> &matches, std::vector<cv::DMatch>& homo_matches);


private:
	cv::Ptr<cv::FeatureDetector> m_detector;
	cv::Ptr<cv::DescriptorExtractor> m_extractor;
	cv::Ptr<cv::DescriptorMatcher> m_matcher;
	double m_ratio;
};

#endif // __ROBUSTMATCHER_H__
