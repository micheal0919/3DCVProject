#include "RobustMatcher.h"

#include <time.h>

// Match feature points using ratio and symmetry test
void CRobustMatcher::RobustMatch(const std::string& image_path_1, const std::string& image_path_2, ImagePairMatch& match)
{
	LOG(INFO) << "Beginning of CRobustMatcher::RobustMatch";

	LOG(INFO) << "image_path_1 = " << image_path_1;
	cv::Mat image_1 = cv::imread(image_path_1);
	CHECK(image_1.data != NULL) << "Fail to read image " << image_path_1;

	//cv::namedWindow("image", cv::WINDOW_AUTOSIZE);
	//cv::imshow("image", image_1);
	//cv::waitKey();

	// 1a. Detection of the ORB features
	std::vector<cv::KeyPoint> keypoints_1;
	this->ComputeKeyPoints(image_1, keypoints_1);
	LOG(INFO) << "The num of key points in image_1 is " << keypoints_1.size();

	// 1b. Extraction of the ORB descriptors
	cv::Mat descriptors_1;
	this->ComputeDescriptors(image_1, keypoints_1, descriptors_1);
	LOG(INFO);

	LOG(INFO) << "image_path_2 = " << image_path_2;
	cv::Mat image_2 = cv::imread(image_path_2, cv::IMREAD_COLOR);
	CHECK(image_2.data != NULL) << "Fail to read image " << image_path_2;
	//cv::namedWindow("image", cv::WINDOW_AUTOSIZE);
	//cv::imshow("image", image_1);
	//cv::waitKey();

	// 1c. Detection of the ORB features
	std::vector<cv::KeyPoint> keypoints_2;
	this->ComputeKeyPoints(image_2, keypoints_2);
	LOG(INFO) << "The num of key points in image_2 is " << keypoints_2.size();

	// 1d. Extraction of the ORB descriptors
	cv::Mat descriptors_2;
	this->ComputeDescriptors(image_2, keypoints_2, descriptors_2);
	LOG(INFO);

	// 2. Match the two image descriptors
	std::vector<std::vector<cv::DMatch> > matches12, matches21;

	// 2a. From image 1 to image 2
	m_matcher->knnMatch(descriptors_1, descriptors_2, matches12, 2); // return 2 nearest neighbours
	LOG(INFO) << "The num of match12 is " << matches12.size();

	// 2b. From image 2 to image 1
	m_matcher->knnMatch(descriptors_2, descriptors_1, matches21, 2); // return 2 nearest neighbours
	LOG(INFO) << "The num of match21 is " << matches21.size();

	// 3. Remove matches for which NN ratio is > than threshold
	// clean image 1 -> image 2 matches
	RatioTest(matches12);
	LOG(INFO) << "The num of match12 is " << matches12.size();

	// clean image 2 -> image 1 matches
	RatioTest(matches21);
	LOG(INFO) << "The num of match21 is " << matches21.size();

	// 4. Remove non-symmetrical matches
	std::vector<cv::DMatch> good_matches;
	SymmetryTest(matches12, matches21, good_matches);
	LOG(INFO) << "The num of good_matches is " << good_matches.size();;

	// use homography to filer bad matches
	std::vector<cv::DMatch> homo_matches;
	HomographyTest(keypoints_1, keypoints_2, good_matches, homo_matches);
	LOG(INFO) << "The num of home_matches is " << homo_matches.size();
 
	//-- Draw only "good" matches
	//cv::Mat img_matches;
	//drawMatches(image_1, keypoints_1, image_2, keypoints_2,
	//	homo_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
	//	std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

	//cv::namedWindow("image", cv::WINDOW_AUTOSIZE);
	//cv::imshow("image", img_matches);
	//cv::waitKey();

	// 5. Set ImagePairMatch 
	match.image1 = image_path_1;
	match.image2 = image_path_2;
	match.correspondences.clear();
	for (size_t i = 0; i < homo_matches.size(); i++)
	{
//		LOG(INFO) << "i = " << i;
		cv::Point2f point1 = keypoints_1[homo_matches[i].queryIdx].pt;
//		LOG(INFO);
		cv::Point2f point2 = keypoints_2[homo_matches[i].trainIdx].pt;
//		LOG(INFO);

		FeatureCorrespondence feature_co;
		//feature_co.feature1(0) = point1.x;
		//feature_co.feature1(1) = point1.y;
		//feature_co.feature2(0) = point2.x;
		//feature_co.feature2(1) = point2.y;
		
		feature_co.feature1.x = point1.x;
		feature_co.feature1.y = point1.y;
		feature_co.feature2.x = point2.x;
		feature_co.feature2.y = point2.y;
//		LOG(INFO);
		match.correspondences.emplace_back(feature_co);
//		LOG(INFO);
	}

	LOG(INFO) << "Ending of CRobustMatcher::RobustMatch";

	//CHECK(0);
}

// Match feature points using ratio test
void CRobustMatcher::FastRobustMatch(const std::string& image_path_1, const std::string& image_path_2, ImagePairMatch& match)
{
	LOG(INFO) << "Beginning of CRobustMatcher::FastRobustMatch";

	LOG(INFO) << "image_path_1 = " << image_path_1;
	cv::Mat image_1 = cv::imread(image_path_1);
	CHECK(image_1.data != NULL) << "Fail to read image " << image_path_1;

	//cv::namedWindow("image", cv::WINDOW_AUTOSIZE);
	//cv::imshow("image", image_1);
	//cv::waitKey();

	// 1a. Detection of the ORB features
	std::vector<cv::KeyPoint> keypoints_1;
	this->ComputeKeyPoints(image_1, keypoints_1);
//	LOG(INFO) << "The num of key points in image_1 is " << keypoints_1.size();

	// 1b. Extraction of the ORB descriptors
	cv::Mat descriptors_1;
	this->ComputeDescriptors(image_1, keypoints_1, descriptors_1);
//	LOG(INFO);

	LOG(INFO) << "image_path_2 = " << image_path_2;
	cv::Mat image_2 = cv::imread(image_path_2, cv::IMREAD_COLOR);
	CHECK(image_2.data != NULL) << "Fail to read image " << image_path_2;
	//cv::namedWindow("image", cv::WINDOW_AUTOSIZE);
	//cv::imshow("image", image_1);
	//cv::waitKey();

	// 1c. Detection of the ORB features
	std::vector<cv::KeyPoint> keypoints_2;
	this->ComputeKeyPoints(image_2, keypoints_2);
//	LOG(INFO) << "The num of key points in image_2 is " << keypoints_2.size();

	// 1d. Extraction of the ORB descriptors
	cv::Mat descriptors_2;
	this->ComputeDescriptors(image_2, keypoints_2, descriptors_2);
	LOG(INFO);

	// 2. Match the two image descriptors
	std::vector<std::vector<cv::DMatch> > matches;
	m_matcher->knnMatch(descriptors_1, descriptors_2, matches, 2);
//	LOG(INFO) << "The num of matches is " << matches.size();

	// 3. Remove matches for which NN ratio is > than threshold
	RatioTest(matches);
//	LOG(INFO) << "The num of matches is " << matches.size();

	// 4. Fill good matches container
	std::vector<cv::DMatch> good_matches;
	for (std::vector<std::vector<cv::DMatch> >::iterator
		matchIterator = matches.begin(); matchIterator != matches.end(); ++matchIterator)
	{
		if (!matchIterator->empty()) good_matches.push_back((*matchIterator)[0]);
	}
	LOG(INFO) << "The num of good matches is " << good_matches.size();

	//-- Draw only "good" matches
	//cv::Mat img_matches;
	//drawMatches(image_1, keypoints_1, image_2, keypoints_2,
	//	good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
	//	std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

	//cv::namedWindow("image", cv::WINDOW_AUTOSIZE);
	//cv::imshow("image", img_matches);
	//cv::waitKey();

	// 5. Set ImagePairMatch 
	match.image1 = image_path_1;
	match.image2 = image_path_2;
	match.correspondences.clear();
	for (size_t i = 0; i < good_matches.size(); i++)
	{
//		LOG(INFO) << "i = " << i;
		cv::Point2f point1 = keypoints_1[good_matches[i].queryIdx].pt;
//		LOG(INFO);
		cv::Point2f point2 = keypoints_2[good_matches[i].trainIdx].pt;
//		LOG(INFO);

		FeatureCorrespondence feature_co;
		//feature_co.feature1(0) = point1.x;
		//feature_co.feature1(1) = point1.y;
		//feature_co.feature2(0) = point2.x;
		//feature_co.feature2(1) = point2.y;

		feature_co.feature1.x = point1.x;
		feature_co.feature1.y = point1.y;
		feature_co.feature2.x = point2.x;
		feature_co.feature2.y = point2.y;
//		LOG(INFO);
		match.correspondences.emplace_back(feature_co);
//		LOG(INFO);
	}
	LOG(INFO) << "Ending of CRobustMatcher::FastRobustMatch";
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

void CRobustMatcher::HomographyTest(const std::vector<cv::KeyPoint>& keypoints_1, const std::vector<cv::KeyPoint>& keypoints_2, 
	const std::vector<cv::DMatch> &matches, std::vector<cv::DMatch>& homo_matches)
{
	if (matches.size() < 10)
	{
		homo_matches = matches;
		LOG(INFO);
		return;
	}
	const double threshold = 100;
	std::vector<cv::Point2d> points_1;
	std::vector<cv::Point2d> points_2;

	for (size_t i = 0; i < matches.size(); i++)
	{
		cv::DMatch match = matches[i];
		cv::Point2d p1 = keypoints_1[match.queryIdx].pt;
		cv::Point2d p2 = keypoints_2[match.trainIdx].pt;
		points_1.emplace_back(p1);
		points_2.emplace_back(p2);
	}

	cv::Mat H = cv::findHomography(points_1, points_2, CV_RANSAC);
	LOG(INFO) << "homegraphy is " << H;

	std::vector<cv::Point2d> estimate_points;
	estimate_points.reserve(matches.size());

	cv::perspectiveTransform(points_1, estimate_points, H);

	homo_matches.clear();
	for (size_t i = 0; i < estimate_points.size(); i++)
	{
		cv::Point2d es_p = estimate_points[i];
		cv::Point2d p2 = points_2[i];
		cv::Point2d diff = p2 - es_p;
		double dis_sqrt = diff.x * diff.x + diff.y * diff.y;

		// if the distance is NOT more than the threshold, add it to the home_matches
		if (dis_sqrt <= threshold)
		{
			homo_matches.emplace_back(matches[i]);
		}
	}

}
