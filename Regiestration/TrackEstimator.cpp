#include "TrackEstimator.h"

#include <glog/logging.h>
#include <opencv2/opencv.hpp>

#include "Track.h"
#include "View.h"
#include "Triangulation.h"

namespace {

void GetObservationsFromTrackViews(
	const TrackId track_id,
	const CReconstruction& reconstruction,
	std::vector<ViewId>* view_ids,
	std::vector<Eigen::Vector3d>* origins,
	std::vector<Eigen::Vector3d>* ray_directions)
{
	const CTrack track = *reconstruction.Track(track_id);
	for (const ViewId view_id : track.ViewIds()) {
		const CView* view = reconstruction.View(view_id);

		// Skip this view if it does not exist or has not been estimated yet.
		if (view == nullptr || !view->IsEstimated()) {
			continue;
		}

		// If the feature is not in the view then we have an ill-formed
		// reconstruction.
		const Feature* feature = CHECK_NOTNULL(view->GetFeature(track_id));
		Eigen::Vector2d pixel(feature->x, feature->y);
		const Eigen::Vector3d image_ray =
			view->Camera().PixelToUnitDepthRay(pixel).normalized();

		view_ids->emplace_back(view_id);
		origins->emplace_back(view->Camera().GetPosition());
		ray_directions->emplace_back(image_ray);
	}
}

// Returns false if the reprojection error of the triangulated point is greater
// than the max allowable reprojection error (for any observation) and true
// otherwise.
bool AcceptableReprojectionError(
	const CReconstruction& reconstruction,
	const TrackId& track_id,
	const double sq_max_reprojection_error_pixels) 
{
	const CTrack& track = *reconstruction.Track(track_id);
	int num_projections = 0;
	double mean_sq_reprojection_error = 0;
	for (const ViewId view_id : track.ViewIds()) {
		const CView* view = reconstruction.View(view_id);
		if (view == nullptr || !view->IsEstimated()) {
			continue;
		}
		const CCamera& camera = view->Camera();
		const Feature* feature = view->GetFeature(track_id);
		Eigen::Vector2d reprojection;
		if (camera.ProjectPoint(track.Point(), &reprojection) < 0) 
		{
			LOG(INFO);
//			return false;
		}
		Eigen::Vector2d pixel(feature->x, feature->y);
		mean_sq_reprojection_error += (pixel - reprojection).squaredNorm();
		++num_projections;
	}

	double error = mean_sq_reprojection_error / static_cast<double>(num_projections);

	LOG(INFO) << "reprojection error is " << error << " of track id " << track_id;

	return (mean_sq_reprojection_error / static_cast<double>(num_projections)) <
		sq_max_reprojection_error_pixels;
}

int NumEstimatedViewsObservingTrack(const CReconstruction& reconstruction,
	const CTrack& track) 
{
	int num_estimated_views = 0;
	for (const ViewId view_id : track.ViewIds()) {
		const CView* view = reconstruction.View(view_id);
		if (view != nullptr && view->IsEstimated()) {
			++num_estimated_views;
		}
	}
	return num_estimated_views;
}

}  // namespace

// Estimate only the tracks supplied by the user.
CTrackEstimator::Summary CTrackEstimator::EstimateAllTracks() 
{
	LOG(INFO) << "CTrackEstimator::EstimateAllTracks()";

	const auto& track_ids = m_reconstruction->TrackIds();
	std::unordered_set<TrackId> tracks(track_ids.begin(), track_ids.end());
	return EstimateTracks(tracks);
}

CTrackEstimator::Summary CTrackEstimator::EstimateTracks(
	const std::unordered_set<TrackId>& track_ids) 
{
	LOG(INFO) << "Beginning of CTrackEstimator::EstimateTracks";

	m_tracks_to_estimate.clear();
	m_successfully_estimated_tracks.clear();

	CTrackEstimator::Summary summary;

	// Get all unestimated track ids.
	m_tracks_to_estimate.reserve(track_ids.size());
	for (const TrackId track_id : track_ids) 
	{
		CTrack* track = m_reconstruction->MutableTrack(track_id);
		if (track->IsEstimated()) {
			++summary.input_num_estimated_tracks;
			continue;
		}

		const int num_views_observing_track =
			NumEstimatedViewsObservingTrack(*m_reconstruction, *track);
		// Skip tracks that do not have enough observations.
		if (num_views_observing_track < 2) {
			continue;
		}
		m_tracks_to_estimate.emplace_back(track_id);
	}
	summary.num_triangulation_attempts = m_tracks_to_estimate.size();

	// Exit early if there are no tracks to estimate.
	if (m_tracks_to_estimate.size() == 0) {
		return summary;
	}

	EstimateTrackSet(0, m_tracks_to_estimate.size());

	// Find the tracks that were newly estimated.
	for (const TrackId track_id : m_tracks_to_estimate) 
	{
		CTrack* track = m_reconstruction->MutableTrack(track_id);
		if (track->IsEstimated()) 
		{
			summary.estimated_tracks.insert(track_id);
		}
	}

	LOG(INFO) << summary.estimated_tracks.size() << " tracks were estimated of "
		<< summary.num_triangulation_attempts << " possible tracks.";

	LOG(INFO) << "Beginning of CTrackEstimator::EstimateTracks";

	return summary;
}

void CTrackEstimator::EstimateTrackSet(const int start, const int end) 
{
	reconstruction_test();

	for (int i = start; i < end; i++) {
		EstimateTrack(m_tracks_to_estimate[i]);
	}
}

bool CTrackEstimator::EstimateTrack(const TrackId track_id) 
{
	LOG(INFO) << "Beginning of CTrackEstimator::EstimateTrack";

	LOG(INFO) << "track id is " << track_id;

	//if (!EstimateTrackMidpoint(track_id))
	//{
	//	LOG(ERROR) << "fail to estimate track with TriangulateMidpoint algorithm";
	//	return false;
	//}

	if (!EstimateTrackNViewSVD(track_id))
	{
		LOG(ERROR) << "fail to estimate track with TriangulateMidpoint algorithm";
//		return false;
	}

	LOG(INFO) << "Endding of CTrackEstimator::EstimateTrack";

	return true;
}

// estimate tracks using TriangulateMidpoint algorithm
bool CTrackEstimator::EstimateTrackMidpoint(const TrackId track_id)
{
	LOG(INFO) << "Beignning of CTrackEstimator::EstimateTrackMidpoint";

	LOG(INFO) << "track id is " << track_id;

	CTrack* track = m_reconstruction->MutableTrack(track_id);
	CHECK(!track->IsEstimated()) << "Track " << track_id
		<< " is already estimated.";

	// Gather projection matrices and features.
	std::vector<ViewId> view_ids;
	std::vector<Eigen::Vector3d> origins, ray_directions;
	GetObservationsFromTrackViews(track_id,
		*m_reconstruction,
		&view_ids,
		&origins,
		&ray_directions);

	// Check the angle between views.
	if (!SufficientTriangulationAngle(ray_directions, m_options.min_triangulation_angle_degrees))
	{
		LOG(WARNING) << "insufficient triangulation angle";
		return false;
	}

	// Triangulate the track.
	if (!TriangulateMidpoint(origins, ray_directions, track->MutablePoint()))
	{
		LOG(WARNING) << "TriangulateMidpoint failed";
		return false;
	}

	// Ensure the reprojection errors are acceptable.
	const double sq_max_reprojection_error_pixels =
		m_options.max_acceptable_reprojection_error_pixels *
		m_options.max_acceptable_reprojection_error_pixels;

	if (!AcceptableReprojectionError(*m_reconstruction,
		track_id,
		sq_max_reprojection_error_pixels)) {
		LOG(WARNING) << "reprojection error is NOT acceptable";
		//		return false;
	}

	track->SetEstimated(true);

	LOG(INFO) << "Endding of CTrackEstimator::EstimateTrackMidpoint";

	return true;
}

// estimate tracks using TriangulateNViewSVD algorithm
bool CTrackEstimator::EstimateTrackNViewSVD(const TrackId track_id)
{
	LOG(INFO) << "Beginning of CTrackEstimator::EstimateTrackNViewSVD";

	std::vector<Matrix3x4d> poses;
	std::vector<Eigen::Vector2d> points;

	CTrack* track = m_reconstruction->MutableTrack(track_id);
	for (const ViewId view_id : track->ViewIds()) 
	{
		const CView* view = m_reconstruction->View(view_id);

		// Skip this view if it does not exist or has not been estimated yet.
		if (view == nullptr || !view->IsEstimated()) {
			continue;
		}


		const Feature* feature = view->GetFeature(track_id);
		CHECK(feature) << "fail to get the featue in this view";

		Eigen::Vector2d point(feature->x, feature->y);
		points.emplace_back(point);

		Matrix3x4d pose;
		view->Camera().GetProjectionMatrix(&pose);
		poses.emplace_back(pose);

	}

	// Triangulate the track.
	if (!TriangulateNViewSVD(poses, points, track->MutablePoint()))
	{
		LOG(WARNING) << "TriangulateMidpoint failed";
		return false;
	}

	// Ensure the reprojection errors are acceptable.
	const double sq_max_reprojection_error_pixels =
		m_options.max_acceptable_reprojection_error_pixels *
		m_options.max_acceptable_reprojection_error_pixels;

	if (!AcceptableReprojectionError(*m_reconstruction,
		track_id,
		sq_max_reprojection_error_pixels)) 
	{
		LOG(WARNING) << "reprojection error is NOT acceptable";
//		return false;
	}

	track->SetEstimated(true);

	LOG(INFO) << "Endding of CTrackEstimator::EstimateTrackNViewSVD";

	return true;
}

// estimate tracks using TriangulateNView algorithm
bool CTrackEstimator::EstimateTrackNView(const TrackId track_id)
{
	LOG(INFO) << "Beginning of CTrackEstimator::EstimateTrackNView";

	std::vector<Matrix3x4d> poses;
	std::vector<Eigen::Vector2d> points;

	CTrack* track = m_reconstruction->MutableTrack(track_id);
	for (const ViewId view_id : track->ViewIds())
	{
		const CView* view = m_reconstruction->View(view_id);

		// Skip this view if it does not exist or has not been estimated yet.
		if (view == nullptr || !view->IsEstimated()) {
			continue;
		}


		const Feature* feature = view->GetFeature(track_id);
		CHECK(feature) << "fail to get the featue in this view";

		Eigen::Vector2d point(feature->x, feature->y);
		points.emplace_back(point);

		Matrix3x4d pose;
		view->Camera().GetProjectionMatrix(&pose);
		poses.emplace_back(pose);

	}

	// Triangulate the track.
	if (!TriangulateNView(poses, points, track->MutablePoint()))
	{
		LOG(WARNING) << "TriangulateMidpoint failed";
		return false;
	}

	// Ensure the reprojection errors are acceptable.
	const double sq_max_reprojection_error_pixels =
		m_options.max_acceptable_reprojection_error_pixels *
		m_options.max_acceptable_reprojection_error_pixels;

	if (!AcceptableReprojectionError(*m_reconstruction,
		track_id,
		sq_max_reprojection_error_pixels))
	{
		LOG(WARNING) << "reprojection error is NOT acceptable";
		//		return false;
	}

	track->SetEstimated(true);

	LOG(INFO) << "Endding of CTrackEstimator::EstimateTrackNView";
	return true;
}

void CTrackEstimator::reconstruction_test()
{
	LOG(INFO) << "Beginnig of CTrackEstimator::reconstruction_test";

	int num_views = m_reconstruction->NumViews();
	LOG(INFO) << "The num of views is " << num_views;

	std::vector<ViewId> view_ids = m_reconstruction->ViewIds();
	for (size_t i = 0; i < view_ids.size(); i++)
	{
		LOG(INFO) << "iterator i = " << i;
		LOG(INFO) << "view is = " << view_ids[i];
		const CView *view = m_reconstruction->View(view_ids[i]);
		LOG(INFO) << "view name is " << view->Name();
		LOG(INFO) << "camera focal_length = " << view->Camera().FocalLength();
		LOG(INFO) << "camera AspectRatio = " << view->Camera().AspectRatio();
		LOG(INFO) << "camera PrincipalPointX = " << view->Camera().PrincipalPointX();
		LOG(INFO) << "camera PrincipalPointY = " << view->Camera().PrincipalPointY();

		LOG(INFO) << "camera postion is " << view->CameraPostion()(0) << " " << view->CameraPostion()(1) << " " << view->CameraPostion()(2);
		LOG(INFO) << "camera ratation matrix is " 
			<< view->CameraOrientationMatrix()(0, 0) << " "
			<< view->CameraOrientationMatrix()(0, 1) << " "
			<< view->CameraOrientationMatrix()(0, 2) << " "
			<< view->CameraOrientationMatrix()(1, 0) << " "
			<< view->CameraOrientationMatrix()(1, 1) << " "
			<< view->CameraOrientationMatrix()(1, 2) << " "
			<< view->CameraOrientationMatrix()(2, 0) << " "
			<< view->CameraOrientationMatrix()(2, 1) << " "
			<< view->CameraOrientationMatrix()(2, 2) << " ";

//		LOG(INFO) << "camera orientation in the view is " << view->CameraOrientationMatrix();
//		LOG(INFO) << "camera postion in the view is " << view->CameraPostion();
		std::vector<TrackId> track_ids = view->TrackIds();
		std::vector<Feature> features;
		for (const TrackId track_id : track_ids)
		{
			const Feature* feature = view->GetFeature(track_id);
			features.emplace_back(*feature);
		}

		cv::Mat img = cv::imread(view->Name());
		
		//cv::namedWindow("imageshow", cv::WINDOW_AUTOSIZE);
		//cv::imshow("imageshow", img);
		//cv::waitKey();

		int myradius = 5;
		for (int i = 0; i < features.size(); i++)
		{
			cv::circle(img, cvPoint(features[i].x, features[i].y), myradius, CV_RGB(100, 0, 0), -1, 8, 0);
		}
			
		//cv::namedWindow("imageshow", cv::WINDOW_AUTOSIZE);
		//cv::imshow("imageshow", img);
		//cv::waitKey();

	}



	LOG(INFO) << "Endding of CTrackEstimator::reconstruction_test";
}