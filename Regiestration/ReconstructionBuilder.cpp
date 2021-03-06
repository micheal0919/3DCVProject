#include "ReconstructionBuilder.h"

#include <glog/logging.h>
#include <opencv2/opencv.hpp>

#include "util/uitl.h"
#include "ImageInfoIO.h"
#include "Reconstruction.h"
#include "TrackBuilder.h"
#include "FeatureExtractorAndMatcher.h"
#include "Track.h"
#include "View.h"
#include "ViewGraph.h"

namespace {


bool AddViewToReconstruction(const std::string& image_filepath,
		const CameraIntrinsicsPrior* intrinsics,
		const Eigen::Matrix3d& camera_orientation_matrix,
		const Eigen::Vector3d& camera_postion,
		CReconstruction* reconstruction) 
{
	LOG(INFO) << "Beginning of AddViewToReconstruction";

	const ViewId view_id = reconstruction->AddView(image_filepath);
	if (view_id == kInvalidViewId) {
		LOG(INFO) << "Could not add " << image_filepath
			<< " to the reconstruction.";
		return false;
	}

	if (intrinsics != nullptr) {
		CView* view = reconstruction->MutableView(view_id);
		*view->MutableCameraIntrinsicsPrior() = *intrinsics;
		*view->MutableCameraOrientationMatrix() = camera_orientation_matrix;
		*view->MutableCameraPostion() = camera_postion;
	}

	LOG(INFO) << "Ending of AddViewToReconstruction";
	return true;
}

CReconstruction* CreateEstimatedSubreconstruction(const CReconstruction& input_reconstruction) 
{
	std::unique_ptr<CReconstruction> subreconstruction(new CReconstruction(input_reconstruction));
	const auto& view_ids = subreconstruction->ViewIds();
	for (const ViewId view_id : view_ids) {
		const CView* view = subreconstruction->View(view_id);
		if (view == nullptr) {
			continue;
		}

		if (!view->IsEstimated()) {
			subreconstruction->RemoveView(view_id);
		}
	}

	const auto& track_ids = subreconstruction->TrackIds();
	for (const TrackId track_id : track_ids) {
		const CTrack* track = subreconstruction->Track(track_id);
		if (track == nullptr) {
			continue;
		}

		if (!track->IsEstimated()) {
			subreconstruction->RemoveTrack(track_id);
		}
	}
	return subreconstruction.release();
}

void RemoveEstimatedViewsAndTracks(CReconstruction* reconstruction, CViewGraph* view_graph) 
{
	LOG(INFO) << "Beginning of RemoveEstimatedViewsAndTracks";

	const auto& view_ids = reconstruction->ViewIds();
	LOG(INFO) << "The num of views is " << view_ids.size();

	for (const ViewId view_id : view_ids) {
		const CView* view = reconstruction->View(view_id);
		if (view == nullptr) {
			continue;
		}

		if (view->IsEstimated()) {
			reconstruction->RemoveView(view_id);
			view_graph->RemoveView(view_id);
		}
	}

	const auto& track_ids = reconstruction->TrackIds();
	LOG(INFO) << "The num of tracks is " << track_ids.size();

	for (const TrackId track_id : track_ids) {
		const CTrack* track = reconstruction->Track(track_id);
		if (track == nullptr) {
			continue;
		}

		if (track->IsEstimated()) {
			reconstruction->RemoveTrack(track_id);
		}
	}

	LOG(INFO) << "Endding of RemoveEstimatedViewsAndTracks";
}

}

CReconstructionBuilder::CReconstructionBuilder(const ReconstructionBuilderOptions& options)
	: m_options(options)
{
	LOG(INFO) << std::endl;
	m_feature_extractor_and_matcher.reset(new CFeatureExtractorAndMatcher(m_options.feam_options));
	LOG(INFO) << std::endl;

	m_view_graph.reset(new CViewGraph());
	LOG(INFO) << std::endl;

	m_reconstruction.reset(new CReconstruction());
	LOG(INFO) << std::endl;

	LOG(INFO) << "m_options.max_track_length = " << m_options.max_track_length;
	m_track_builder.reset(new CTrackBuilder(m_options.max_track_length));
	LOG(INFO) << std::endl;
}


CReconstructionBuilder::~CReconstructionBuilder()
{
}

bool CReconstructionBuilder::AddImages()
{
	LOG(INFO) << "Beginning of CReconstructionBuilder::AddImages";

	LOG(INFO) << "m_options.image_info_file_path = " << m_options.image_info_file_path;
	std::vector<ImageInfo> infos;
	ReadImageInfoFromFile(m_options.image_info_file_path, infos);
	CHECK_EQ(infos.size(), m_options.num_image) << "The image num got from xml is not equal to num of option";
	
	for (int i = 0; i < m_options.num_image; i++)
	{

		std::string image_name = GetImageNameFromNum(i);
		std::string image_path = m_options.image_path + image_name;
		LOG(INFO) << "image_path = " << image_path;

		m_image_filepaths.emplace_back(image_path);

		CameraIntrinsicsPrior intrinsics;
		Eigen::Matrix3d camera_ration_matrix;
		Eigen::Vector3d camera_postion;
		ImageInfoToCameraInfo(infos[i], intrinsics, camera_ration_matrix, camera_postion);
		CHECK(AddViewToReconstruction(image_path, &intrinsics, camera_ration_matrix, camera_postion, m_reconstruction.get()));

		//cv::Mat img = cv::imread(image_path);
		//cv::Size size(infos[i].image_width, infos[i].image_height);
		//cv::resize(img, img, size);
		//image_name = "sfm" + image_name;
		//image_path = m_options.image_path + image_name;
		//cv::imwrite(image_path, img);

		m_feature_extractor_and_matcher->AddImage(image_path, intrinsics);
	}
	
	CHECK_EQ(m_reconstruction->NumViews(), m_options.num_image) << "The num views in view graph is not equal to the num of image files" << std::endl;

	LOG(INFO) << "Ending of CReconstructionBuilder::AddImages";
	return true;
}

bool CReconstructionBuilder::ExtractAndMatchFeatures()
{
	LOG(INFO) << "Beginning of CReconstructionBuilder::ExtractAndMatchFeatures";
	CHECK_EQ(m_view_graph->NumViews(), 0) << "Cannot call ExtractAndMatchFeatures "
		"after TwoViewMatches has been "
		"called.";
	LOG(INFO);

	std::vector<ImagePairMatch> matches;
	m_feature_extractor_and_matcher->ExtractAndMatchFeatures(matches);
	LOG(INFO);

	const size_t num_total_view_pairs = m_image_filepaths.size() * (m_image_filepaths.size() - 1) / 2;
	LOG(INFO) << matches.size() << " of " << num_total_view_pairs
		<< " view pairs were matched and geometrically verified.";

	for (const auto& match : matches) {
		AddTwoViewMatch(match.image1, match.image2, match);
	}

	LOG(INFO) << "Ending of CReconstructionBuilder::ExtractAndMatchFeatures";
	return true;
}

bool CReconstructionBuilder::BuildReconstruction(std::vector<CReconstruction*>* reconstructions)
{
	LOG(INFO) << "Beginning of CReconstructionBuilder::BuildReconstruction";

	CHECK_GE(m_view_graph->NumViews(), 2) << "At least 2 images must be provided "
		"in order to create a "
		"reconstruction.";

	if (m_reconstruction->NumTracks() == 0) 
	{
		m_track_builder->BuildTracks(m_reconstruction.get());
	}
	LOG(INFO) << "Suceed to build all tracks, and the track num is " << m_reconstruction->NumTracks();

	if (m_options.only_calibrated_views) {
		LOG(INFO) << "Removing uncalibrated views.";
//		RemoveUncalibratedViews();
	}

	while (m_reconstruction->NumViews() > 1) {
		LOG(INFO) << "Attempting to reconstruct " << m_reconstruction->NumViews()
			<< " images from " << m_view_graph->NumEdges()
			<< " two view matches.";

		std::unique_ptr<CReconstructionEstimator> reconstruction_estimator(
			CReconstructionEstimator::Create(
			m_options.reconstruction_estimator_options));
		
		LOG(INFO);

		const auto& summary = reconstruction_estimator->Estimate(m_view_graph.get(), m_reconstruction.get());

		// If a reconstruction can no longer be estimated, return.
		if (!summary.success) {
			return reconstructions->size() > 0;
		}

		LOG(INFO)
			<< "\nReconstruction estimation statistics: "
			<< "\n\tNum estimated views = " << summary.estimated_views.size()
			<< "\n\tNum input views = " << m_reconstruction->NumViews()
			<< "\n\tNum estimated tracks = " << summary.estimated_tracks.size()
			<< "\n\tNum input tracks = " << m_reconstruction->NumTracks()
			<< "\n\tPose estimation time = " << summary.pose_estimation_time
			<< "\n\tTriangulation time = " << summary.triangulation_time
			<< "\n\tBundle Adjustment time = " << summary.bundle_adjustment_time
			<< "\n\tTotal time = " << summary.total_time
			<< "\n\n" << summary.message;

		reconstructions->emplace_back(CreateEstimatedSubreconstruction(*m_reconstruction));
		RemoveEstimatedViewsAndTracks(m_reconstruction.get(), m_view_graph.get());
		LOG(INFO);

		// Exit after the first reconstruction estimation if only the single largest
		// reconstruction is desired.
		//if (m_options.reconstruct_largest_connected_component) {
		//	return reconstructions->size() > 0;
		//}

		if (m_reconstruction->NumViews() < 3) {
			LOG(INFO) << "No more reconstructions can be estimated.";
			return reconstructions->size() > 0;
		}
		LOG(INFO);

	}

	LOG(INFO) << "Endding of CReconstructionBuilder::BuildReconstruction";

	return true;

}

bool CReconstructionBuilder::AddTwoViewMatch(const std::string& image1,
	const std::string& image2,
	const ImagePairMatch& matches)
{
	LOG(INFO) << "Beginning of CReconstructionBuilder::AddTwoViewMatch";

	const ViewId view_id1 = m_reconstruction->ViewIdFromName(image1);
	const ViewId view_id2 = m_reconstruction->ViewIdFromName(image2);
	CHECK_NE(view_id1, kInvalidViewId)
		<< "Tried to add a view with the name " << image1
		<< " to the view graph but does not exist in the reconstruction.";
	CHECK_NE(view_id2, kInvalidViewId)
		<< "Tried to add a view with the name " << image2
		<< " to the view graph but does not exist in the reconstruction.";

	const CView* view1 = m_reconstruction->View(view_id1);
	const CView* view2 = m_reconstruction->View(view_id2);
	if (m_options.only_calibrated_views &&
		(!view1->CameraIntrinsicsPrior().focal_length.is_set ||
		!view2->CameraIntrinsicsPrior().focal_length.is_set)) {
		return false;
	}

	AddMatchToViewGraph(view_id1, view_id2, matches);
	AddTracksForMatch(view_id1, view_id2, matches);

	LOG(INFO) << "Endding of CReconstructionBuilder::AddTwoViewMatch";

	return true;
}

void CReconstructionBuilder::AddMatchToViewGraph(
	const ViewId view_id1,
	const ViewId view_id2,
	const ImagePairMatch& image_matches) 
{
	LOG(INFO) << "Beginning of CReconstructionBuilder::AddMatchToViewGraph";

	TwoViewInfo twoview_info = image_matches.twoview_info;
	if (view_id1 > view_id2) {
		SwapCameras(&twoview_info);
	}

	m_view_graph->AddEdge(view_id1, view_id2, twoview_info);

	LOG(INFO) << "Endding of CReconstructionBuilder::AddMatchToViewGraph";
}

void CReconstructionBuilder::AddTracksForMatch(const ViewId view_id1,
	const ViewId view_id2,
	const ImagePairMatch& matches) 
{
	LOG(INFO) << "Beginning of CReconstructionBuilder::AddTracksForMatch";

	for (const auto& match : matches.correspondences) 
	{
		m_track_builder->AddFeatureCorrespondence(view_id1, match.feature1,
			view_id2, match.feature2);
	}

	LOG(INFO) << "Endding of CReconstructionBuilder::AddTracksForMatch";
}
