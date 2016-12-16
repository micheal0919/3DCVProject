#include "FeatureExtractorAndMatcher.h"
#include <glog/logging.h>

CFeatureExtractorAndMatcher::CFeatureExtractorAndMatcher(const Options& options)
	: m_options(options)
{
	LOG(INFO) << "m_options.num_threads = " << m_options.num_threads;
	LOG(INFO) << "m_options.num_keypoint = " << m_options.num_keypoint;
	LOG(INFO) << "m_options.lsh_index_params_1 = " << m_options.lsh_index_params_1;
	LOG(INFO) << "m_options.lsh_index_params_2 = " << m_options.lsh_index_params_2;
	LOG(INFO) << "m_options.lsh_index_params_3 = " << m_options.lsh_index_params_3;
	LOG(INFO) << "m_options.search_params = " << m_options.search_params;
	LOG(INFO) << "m_options.ratio = " << m_options.ratio;
	LOG(INFO) << "m_options.symmetry_match = " << m_options.symmetry_match;
	LOG(INFO) << "m_options.max_num_features = " << m_options.max_num_features;
	LOG(INFO) << "m_options.min_num_inlier_matches = " << m_options.min_num_inlier_matches;

	m_matcher.reset(new CRobustMatcher);
	LOG(INFO) << std::endl;

	//cv::Ptr<cv::FeatureDetector> orb = cv::ORB::create();
	//LOG(INFO) << std::endl;

	//// set feature detector
	//m_matcher->SetFeatureDetector(orb);
	//// set descriptor extractor
	//m_matcher->SetDescriptorExtractor(orb);
	//LOG(INFO) << std::endl;

	//// instantiate LSH index parameters
	//cv::Ptr<cv::flann::IndexParams> indexParams = cv::makePtr<cv::flann::LshIndexParams>(m_options.lsh_index_params_1, 
	//	m_options.lsh_index_params_2, m_options.lsh_index_params_3);
	//// instantiate flann search parameters
	//cv::Ptr<cv::flann::SearchParams> searchParams = cv::makePtr<cv::flann::SearchParams>(m_options.search_params);
	//LOG(INFO) << std::endl;

	//// instantiate FlannBased matcher
	//cv::Ptr<cv::DescriptorMatcher> matcher = cv::makePtr<cv::FlannBasedMatcher>(indexParams, searchParams);
	//LOG(INFO) << std::endl;
	//
	//// set matcher
	//m_matcher->SetDescriptorMatcher(matcher);
	//LOG(INFO) << std::endl;

	//// set ratio test parameter
	//m_matcher->SetRatio(m_options.ratio); 
	//LOG(INFO) << std::endl;
}


CFeatureExtractorAndMatcher::~CFeatureExtractorAndMatcher()
{
}

// Add all images with known camera projection to the image matcher queue.
//bool CFeatureExtractorAndMatcher::AddAllImages(const std::vector<std::string>& image_filepaths, 
//	std::unordered_map<std::string, CameraIntrinsicsPrior>& camera_projections)
//{
//	m_image_filepaths = image_filepaths;
//	m_camera_projections = camera_projections;
//
//	// If SetImagePairsToMatch has not been called, match all image-to-image
//	// pairs.
//	m_pairs_to_match.clear();
//
//	// Compute the total number of potential matches.
//	const size_t num_pairs_to_match = m_image_filepaths.size() * (m_image_filepaths.size() - 1) / 2;
//	m_pairs_to_match.reserve(num_pairs_to_match);
//
//	// Create a list of all possible image pairs.
//	for (size_t i = 0; i < m_image_filepaths.size() - 1; i++)
//	{
//		for (size_t j = i + 1; j < m_image_filepaths.size(); j++)
//		{
//			m_pairs_to_match.emplace_back(m_image_filepaths[i], m_image_filepaths[j]);
//		}
//	}
//
//	return true;
//}

bool CFeatureExtractorAndMatcher::AddImage(
	const std::string& image_filepath,
	const CameraIntrinsicsPrior& intrinsics) 
{
	m_image_filepaths.emplace_back(image_filepath);
	m_intrinsics[image_filepath] = intrinsics;
	return true;
}

void CFeatureExtractorAndMatcher::ExtractAndMatchFeatures(std::vector<ImagePairMatch>& matches)
{
	LOG(INFO) << "Beginning of CFeatureExtractorAndMatcher::ExtractAndMatchFeatures";

	// Compute the total number of potential matches.
	m_pairs_to_match.clear();
	const size_t num_pairs_to_match = m_image_filepaths.size() * (m_image_filepaths.size() - 1) / 2;
	m_pairs_to_match.reserve(num_pairs_to_match);
	LOG(INFO);

	// Create a list of all possible image pairs.
	for (size_t i = 0; i < m_image_filepaths.size() - 1; i++)
	{
		for (size_t j = i + 1; j < m_image_filepaths.size(); j++)
		{
			m_pairs_to_match.emplace_back(m_image_filepaths[i], m_image_filepaths[j]);
		}
	}
	LOG(INFO) << "m_pairs_to_match.size() = " << m_pairs_to_match.size();

	// match all the pairs 
	matches.clear();
	for (size_t i = 0; i < m_pairs_to_match.size(); i++)
	{
		std::string image_path_1 = m_pairs_to_match[i].first;
		std::string image_path_2 = m_pairs_to_match[i].second;

		ImagePairMatch match;
		if (m_options.symmetry_match)
		{
			m_matcher->RobustMatch(image_path_1, image_path_2, match);
		}
		else
		{
			m_matcher->FastRobustMatch(image_path_1, image_path_2, match);
		}
		LOG(INFO);

		matches.emplace_back(match);
	}

	LOG(INFO) << "Ending of CFeatureExtractorAndMatcher::ExtractAndMatchFeatures";
}

