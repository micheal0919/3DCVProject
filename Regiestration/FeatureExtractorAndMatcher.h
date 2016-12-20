#ifndef __FEATURE_EXTRACTOR_AND_MATCHER_H__
#define __FEATURE_EXTRACTOR_AND_MATCHER_H__

#include <vector>
#include <string>
#include <unordered_map>
#include <memory>

#include <opencv2/opencv.hpp>

#include "Types.h"
#include "RobustMatcher.h"


class CFeatureExtractorAndMatcher
{
public:


	struct Options {
		int num_threads = 1;
		int num_keypoint = 200;
		// LshIndexParams
		int lsh_index_params_1 = 6;
		int lsh_index_params_2 = 12;
		int lsh_index_params_3 = 1;

		// SearchParams
		int search_params = 50;

		// max ratio between 1st and 2nd NN
		double ratio = 0.7;

		bool symmetry_match = true;
		int max_num_features = 16384;
		int min_num_inlier_matches = 30;

		VerifyTwoViewMatchesOptions geometric_verification_options;

	};


	explicit CFeatureExtractorAndMatcher(const Options& options);
	~CFeatureExtractorAndMatcher();

	// Add an image with known camera intrinsics to the image matcher queue.
	bool AddImage(const std::string& image_filepath, const CameraIntrinsicsPrior& intrinsics);

	// TODO: Only matches that have passed geometric verification are kept. 
	void ExtractAndMatchFeatures(std::vector<ImagePairMatch>& matches);

private:
	const Options m_options;
	std::vector<std::string> m_image_filepaths;
	std::unordered_map<std::string, CameraIntrinsicsPrior> m_intrinsics;
	std::unique_ptr<CRobustMatcher> m_matcher;
	std::vector<std::pair<std::string, std::string> > m_pairs_to_match;

};

#endif // __FEATURE_EXTRACTOR_AND_MATCHER_H__
