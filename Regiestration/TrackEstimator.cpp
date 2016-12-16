#include "TrackEstimator.h"

#include <glog/logging.h>

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
		if (camera.ProjectPoint(track.Point(), &reprojection) < 0) {
			return false;
		}
		Eigen::Vector2d pixel(feature->x, feature->y);
		mean_sq_reprojection_error += (pixel - reprojection).squaredNorm();
		++num_projections;
	}

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
	const auto& track_ids = m_reconstruction->TrackIds();
	std::unordered_set<TrackId> tracks(track_ids.begin(), track_ids.end());
	return EstimateTracks(tracks);
}

CTrackEstimator::Summary CTrackEstimator::EstimateTracks(
	const std::unordered_set<TrackId>& track_ids) 
{
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

	// Estimate the tracks in parallel. Instead of 1 threadpool worker per track,
	// we let each worker estimate a fixed number of tracks at a time (e.g. 20
	// tracks). Since estimating the tracks is so fast, this strategy is better
	// helps speed up multithreaded estimation by reducing the overhead of
	// starting/stopping threads.
	//const int num_threads = std::min(
	//	m_options.num_threads, static_cast<int>(m_tracks_to_estimate.size()));
	//const int interval_step =
	//	std::min(m_options.multithreaded_step_size,
	//	static_cast<int>(m_tracks_to_estimate.size()) / num_threads);

	//std::unique_ptr<ThreadPool> pool(new ThreadPool(num_threads));
	//for (int i = 0; i < tracks_to_estimate_.size(); i += interval_step) {
	//	const int end_interval = std::min(
	//		static_cast<int>(tracks_to_estimate_.size()), i + interval_step);
	//	pool->Add(&TrackEstimator::EstimateTrackSet, this, i, end_interval);
	//}

	//// Wait for all tracks to be estimated.
	//pool.reset(nullptr);

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
	return summary;
}

void CTrackEstimator::EstimateTrackSet(const int start, const int end) 
{
	for (int i = start; i < end; i++) {
		EstimateTrack(m_tracks_to_estimate[i]);
	}
}

bool CTrackEstimator::EstimateTrack(const TrackId track_id) 
{
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
	if (!SufficientTriangulationAngle(ray_directions,
		m_options.min_triangulation_angle_degrees)) {
		return false;
	}

	// Triangulate the track.
	if (!TriangulateMidpoint(origins, ray_directions, track->MutablePoint())) {
		return false;
	}

	// Bundle adjust the track. The 2-view triangulation method is optimal so we
	// do not need to run BA for that case.
	//if (options_.bundle_adjustment) {
	//	track->SetEstimated(true);
	//	const BundleAdjustmentSummary summary =
	//		BundleAdjustTrack(options_.ba_options, track_id, reconstruction_);
	//	track->SetEstimated(false);
	//	if (!summary.success) {
	//		return false;
	//	}
	//}

	// Ensure the reprojection errors are acceptable.
	const double sq_max_reprojection_error_pixels =
		m_options.max_acceptable_reprojection_error_pixels *
		m_options.max_acceptable_reprojection_error_pixels;

	if (!AcceptableReprojectionError(*m_reconstruction,
		track_id,
		sq_max_reprojection_error_pixels)) {
		return false;
	}

	track->SetEstimated(true);
	return true;
}
