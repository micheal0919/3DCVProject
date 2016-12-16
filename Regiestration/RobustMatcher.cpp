#include "RobustMatcher.h"

#include <time.h>

// Match feature points using ratio and symmetry test
void CRobustMatcher::RobustMatch(const std::string& image_path_1, const std::string& image_path_2, ImagePairMatch& match)
{
	LOG(INFO) << "Beginning of CRobustMatcher::RobustMatch";

	LOG(INFO) << "image_path_1 = " << image_path_1;
	cv::Mat image_1 = cv::imread(image_path_1);
	CHECK(image_1.data != NULL) << "Fail to read image " << image_path_1;

	cv::namedWindow("image", cv::WINDOW_AUTOSIZE);
	cv::imshow("image", image_1);

	// 1a. Detection of the ORB features
	std::vector<cv::KeyPoint> keypoints_1;
	this->ComputeKeyPoints(image_1, keypoints_1);
	LOG(INFO);

	// 1b. Extraction of the ORB descriptors
	cv::Mat descriptors_1;
	this->ComputeDescriptors(image_1, keypoints_1, descriptors_1);
	LOG(INFO);

	cv::Mat image_2 = cv::imread(image_path_2, cv::IMREAD_COLOR);
	// 1c. Detection of the ORB features
	std::vector<cv::KeyPoint> keypoints_2;
	this->ComputeKeyPoints(image_2, keypoints_2);

	// 1d. Extraction of the ORB descriptors
	cv::Mat descriptors_2;
	this->ComputeDescriptors(image_2, keypoints_2, descriptors_2);
	LOG(INFO);

	// 2. Match the two image descriptors
	std::vector<std::vector<cv::DMatch> > matches12, matches21;

	// 2a. From image 1 to image 2
	m_matcher->knnMatch(descriptors_1, descriptors_2, matches12, 2); // return 2 nearest neighbours
	LOG(INFO);

	// 2b. From image 2 to image 1
	m_matcher->knnMatch(descriptors_2, descriptors_1, matches21, 2); // return 2 nearest neighbours
	LOG(INFO);

	// 3. Remove matches for which NN ratio is > than threshold
	// clean image 1 -> image 2 matches
	RatioTest(matches12);
	LOG(INFO);

	// clean image 2 -> image 1 matches
	RatioTest(matches21);
	LOG(INFO);

	// 4. Remove non-symmetrical matches
	std::vector<cv::DMatch> good_matches;
	SymmetryTest(matches12, matches21, good_matches);
	LOG(INFO);

	// 5. Set ImagePairMatch 
	match.image1 = image_path_1;
	match.image2 = image_path_2;
	match.correspondences.clear();
	for (size_t i = 0; i < good_matches.size(); i++)
	{
		cv::Point2f point1 = keypoints_2[good_matches[i].trainIdx].pt;
		cv::Point2f point2 = keypoints_2[good_matches[i].queryIdx].pt; 
		FeatureCorrespondence feature_co;
		//feature_co.feature1(0) = point1.x;
		//feature_co.feature1(1) = point1.y;
		//feature_co.feature2(0) = point2.x;
		//feature_co.feature2(1) = point2.y;
		
		feature_co.feature1.x = point1.x;
		feature_co.feature1.y = point1.y;
		feature_co.feature2.x = point2.x;
		feature_co.feature2.y = point2.y;

		match.correspondences.emplace_back(feature_co);
	}

	LOG(INFO) << "Ending of CRobustMatcher::RobustMatch";
}

// Match feature points using ratio test
void CRobustMatcher::FastRobustMatch(const std::string& image_path_1, const std::string& image_path_2, ImagePairMatch& match)
{
	cv::Mat image_1 = cv::imread(image_path_1, cv::IMREAD_COLOR);

	// 1a. Detection of the ORB features
	std::vector<cv::KeyPoint> keypoints_1;
	this->ComputeKeyPoints(image_1, keypoints_1);

	// 1b. Extraction of the ORB descriptors
	cv::Mat descriptors_1;
	this->ComputeDescriptors(image_1, keypoints_1, descriptors_1);


	cv::Mat image_2 = cv::imread(image_path_2, cv::IMREAD_COLOR);
	// 1c. Detection of the ORB features
	std::vector<cv::KeyPoint> keypoints_2;
	this->ComputeKeyPoints(image_2, keypoints_2);

	// 1d. Extraction of the ORB descriptors
	cv::Mat descriptors_2;
	this->ComputeDescriptors(image_2, keypoints_2, descriptors_2);

	// 2. Match the two image descriptors
	std::vector<std::vector<cv::DMatch> > matches;
	m_matcher->knnMatch(descriptors_1, descriptors_2, matches, 2);

	// 3. Remove matches for which NN ratio is > than threshold
	RatioTest(matches);

	// 4. Fill good matches container
	std::vector<cv::DMatch> good_matches;
	for (std::vector<std::vector<cv::DMatch> >::iterator
		matchIterator = matches.begin(); matchIterator != matches.end(); ++matchIterator)
	{
		if (!matchIterator->empty()) good_matches.push_back((*matchIterator)[0]);
	}

	// 5. Set ImagePairMatch 
	match.image1 = image_path_1;
	match.image2 = image_path_2;
	match.correspondences.clear();
	for (size_t i = 0; i < good_matches.size(); i++)
	{
		cv::Point2f point1 = keypoints_2[good_matches[i].trainIdx].pt;
		cv::Point2f point2 = keypoints_2[good_matches[i].queryIdx].pt;
		FeatureCorrespondence feature_co;
		//feature_co.feature1(0) = point1.x;
		//feature_co.feature1(1) = point1.y;
		//feature_co.feature2(0) = point2.x;
		//feature_co.feature2(1) = point2.y;

		feature_co.feature1.x = point1.x;
		feature_co.feature1.y = point1.y;
		feature_co.feature2.x = point2.x;
		feature_co.feature2.y = point2.y;

		match.correspondences.emplace_back(feature_co);
	}
}

void CRobustMatcher::ComputeKeyPoints(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints)
{
	m_detector->detect(image, keypoints);
}

void CRobustMatcher::ComputeDescriptors(const cv::Mat& image, std::vector<cv::KeyPoint>& keypoints, cv::Mat& descriptors)
{
	m_extractor->compute(image, keypoints, descriptors);
}

int CRobustMatcher::RatioTest(std::vector<std::vector<cv::DMatch> > &matches)
{
	int removed = 0;
	// for all matches
	for (std::vector<std::vector<cv::DMatch> >::iterator
		matchIterator = matches.begin(); matchIterator != matches.end(); ++matchIterator)
	{
		// if 2 NN has been identified
		if (matchIterator->size() > 1)
		{
			// check distance ratio
			if ((*matchIterator)[0].distance / (*matchIterator)[1].distance > m_ratio)
			{
				matchIterator->clear(); // remove match
				removed++;
			}
		}
		else
		{ // does not have 2 neighbours
			matchIterator->clear(); // remove match
			removed++;
		}
	}
	return removed;
}

void CRobustMatcher::SymmetryTest(const std::vector<std::vector<cv::DMatch> >& matches1,
	const std::vector<std::vector<cv::DMatch> >& matches2,
	std::vector<cv::DMatch>& symMatches)
{

	// for all matches image 1 -> image 2
	for (std::vector<std::vector<cv::DMatch> >::const_iterator
		matchIterator1 = matches1.begin(); matchIterator1 != matches1.end(); ++matchIterator1)
	{

		// ignore deleted matches
		if (matchIterator1->empty() || matchIterator1->size() < 2)
			continue;

		// for all matches image 2 -> image 1
		for (std::vector<std::vector<cv::DMatch> >::const_iterator
			matchIterator2 = matches2.begin(); matchIterator2 != matches2.end(); ++matchIterator2)
		{
			// ignore deleted matches
			if (matchIterator2->empty() || matchIterator2->size() < 2)
				continue;

			// Match symmetry test
			if ((*matchIterator1)[0].queryIdx ==
				(*matchIterator2)[0].trainIdx &&
				(*matchIterator2)[0].queryIdx ==
				(*matchIterator1)[0].trainIdx)
			{
				// add symmetrical match
				symMatches.push_back(
					cv::DMatch((*matchIterator1)[0].queryIdx,
					(*matchIterator1)[0].trainIdx,
					(*matchIterator1)[0].distance));
				break; // next match in image 1 -> image 2
			}
		}
	}

}

void CRobustMatcher::RobustMatch(const cv::Mat& frame, std::vector<cv::DMatch>& good_matches,
	std::vector<cv::KeyPoint>& keypoints_frame, const cv::Mat& descriptors_model)
{

	// 1a. Detection of the ORB features
	this->ComputeKeyPoints(frame, keypoints_frame);

	// 1b. Extraction of the ORB descriptors
	cv::Mat descriptors_frame;
	this->ComputeDescriptors(frame, keypoints_frame, descriptors_frame);

	// 2. Match the two image descriptors
	std::vector<std::vector<cv::DMatch> > matches12, matches21;

	// 2a. From image 1 to image 2
	m_matcher->knnMatch(descriptors_frame, descriptors_model, matches12, 2); // return 2 nearest neighbours

	// 2b. From image 2 to image 1
	m_matcher->knnMatch(descriptors_model, descriptors_frame, matches21, 2); // return 2 nearest neighbours

	// 3. Remove matches for which NN ratio is > than threshold
	// clean image 1 -> image 2 matches
	RatioTest(matches12);
	// clean image 2 -> image 1 matches
	RatioTest(matches21);

	// 4. Remove non-symmetrical matches
	SymmetryTest(matches12, matches21, good_matches);

}

void CRobustMatcher::FastRobustMatch(const cv::Mat& frame, std::vector<cv::DMatch>& good_matches,
	std::vector<cv::KeyPoint>& keypoints_frame,
	const cv::Mat& descriptors_model)
{
	good_matches.clear();

	// 1a. Detection of the ORB features
	this->ComputeKeyPoints(frame, keypoints_frame);

	// 1b. Extraction of the ORB descriptors
	cv::Mat descriptors_frame;
	this->ComputeDescriptors(frame, keypoints_frame, descriptors_frame);

	// 2. Match the two image descriptors
	std::vector<std::vector<cv::DMatch> > matches;
	m_matcher->knnMatch(descriptors_frame, descriptors_model, matches, 2);

	// 3. Remove matches for which NN ratio is > than threshold
	RatioTest(matches);

	// 4. Fill good matches container
	for (std::vector<std::vector<cv::DMatch> >::iterator
		matchIterator = matches.begin(); matchIterator != matches.end(); ++matchIterator)
	{
		if (!matchIterator->empty()) good_matches.push_back((*matchIterator)[0]);
	}

}