#include "test.h"

#include <tinyxml.h>
#include <opencv2/opencv.hpp>
#include <opencv2/nonfree/features2d.hpp>
#include <glog/logging.h>

#include "RobustMatcher.h"
#include "TrackBuildTest.h"
#include "CameraTest.h"

Test::Test()
{
}


Test::~Test()
{
}

void Test::TestAll()
{
//	CHECK(surf_test()) << "surf_test error";
//	CHECK(tiny_xml_test()) << "tiny xml test error";
//	CHECK(robust_matcher_test()) << "robust matcher test error";
//	CHECK(track_build_test()) << "track builder test error";
	CHECK(camera_test()) << "camera test error";
}

bool Test::surf_test()
{
	std::string img_path_1 = "../data/images/000000.jpg";
	std::string img_path_2 = "../data/images/000001.jpg";
	cv::Mat img_1 = cv::imread(img_path_1);
	cv::Mat img_2 = cv::imread(img_path_2);

	if (!img_1.data || !img_2.data)
	{
		LOG(ERROR) << " --(!) Error reading images \n"; 
		return false;
	}

	//-- Step 1: Detect the keypoints using SURF Detector
	int minHessian = 400;

	cv::SurfFeatureDetector detector(minHessian);

	std::vector<cv::KeyPoint> keypoints_1, keypoints_2;

	detector.detect(img_1, keypoints_1);
	detector.detect(img_2, keypoints_2);

	//-- Step 2: Calculate descriptors (feature vectors)
	cv::SurfDescriptorExtractor extractor;

	cv::Mat descriptors_1, descriptors_2;

	extractor.compute(img_1, keypoints_1, descriptors_1);
	extractor.compute(img_2, keypoints_2, descriptors_2);

	//-- Step 3: Matching descriptor vectors using FLANN matcher
	cv::FlannBasedMatcher matcher;
	std::vector< cv::DMatch > matches;
	matcher.match(descriptors_1, descriptors_2, matches);

	double max_dist = 0; double min_dist = 100;

	//-- Quick calculation of max and min distances between keypoints
	for (int i = 0; i < descriptors_1.rows; i++)
	{
		double dist = matches[i].distance;
		if (dist < min_dist) min_dist = dist;
		if (dist > max_dist) max_dist = dist;
	}

	printf("-- Max dist : %f \n", max_dist);
	printf("-- Min dist : %f \n", min_dist);

	//-- Draw only "good" matches (i.e. whose distance is less than 2*min_dist,
	//-- or a small arbitary value ( 0.02 ) in the event that min_dist is very
	//-- small)
	//-- PS.- radiusMatch can also be used here.
	std::vector< cv::DMatch > good_matches;

	for (int i = 0; i < descriptors_1.rows; i++)
	{
		if (matches[i].distance <= cv::max(2 * min_dist, 0.02))
		{
			good_matches.push_back(matches[i]);
		}
	}

	//-- Draw only "good" matches
	cv::Mat img_matches;
	drawMatches(img_1, keypoints_1, img_2, keypoints_2,
		good_matches, img_matches, cv::Scalar::all(-1), cv::Scalar::all(-1),
		std::vector<char>(), cv::DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

	//-- Show detected matches
	imshow("Good Matches", img_matches);

	for (int i = 0; i < (int)good_matches.size(); i++)
	{
		printf("-- Good Match [%d] Keypoint 1: %d  -- Keypoint 2: %d  \n", i, good_matches[i].queryIdx, good_matches[i].trainIdx);
	}

	cv::waitKey(0);

	return true;
}

bool Test::tiny_xml_test()
{
	TiXmlDocument* myDocument = new TiXmlDocument();
	TiXmlDeclaration* decl = new TiXmlDeclaration("1.0", "UTF-8", "");
	myDocument->LinkEndChild(decl);

	TiXmlElement* RootElement = new TiXmlElement("ImageInfos");
	myDocument->LinkEndChild(RootElement);

	// make double more prcise
	const int magnitude = 10000;
	RootElement->SetAttribute("magnitude", magnitude);

	myDocument->SaveFile("output.xml");

	return true;
}

bool Test::robust_matcher_test()
{
	std::string img_path_1 = "../data/images/000000.jpg";
	std::string img_path_2 = "../data/images/000002.jpg";

	CRobustMatcher matcher;
	ImagePairMatch match;
	matcher.RobustMatch(img_path_1, img_path_2, match);
	LOG(INFO) << "The robust matched feature num is " << match.correspondences.size();

	matcher.FastRobustMatch(img_path_1, img_path_2, match);
	LOG(INFO) << "The faset robust matched feature num is " << match.correspondences.size();

	return true;
}

bool Test::track_build_test()
{
	CTrackBuildTest test;
	return test.Test();
}

bool Test::camera_test()
{
	CCameraTest test;
	return test.Test();
}