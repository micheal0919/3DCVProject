#ifndef __TRACK_ESTIMATOR_H__
#define __TRACK_ESTIMATOR_H__

#include <unordered_set>
#include <memory>

#include "Types.h"
#include "Reconstruction.h"

class CTrackEstimator
{
public:
	struct Options {
		// Number of threads for multithreading.
		int num_threads = 1;

		// Maximum reprojection error for successful triangulation.
		double max_acceptable_reprojection_error_pixels = 5.0;

		// Minimum triangulation angle between two views required for
		// triangulation. For N-View triangulation we require that at least one pair
		// of views has this an angle this large.
		double min_triangulation_angle_degrees = 3.0;

		// Perform bundle adjustment on the track as soon as a position is
		// estimated.
		bool bundle_adjustment = false;
		BundleAdjustmentOptions ba_options;

		// For thread-level parallelism, it is better to estimate a small fixed
		// number of tracks per thread worker instead of 1 track per worker. This
		// number controls how many points are estimated per worker.
		int multithreaded_step_size = 100;
	};

	struct Summary {
		// Number of estimated tracks that were input.
		int input_num_estimated_tracks = 0;

		// Number of estimated tracks after trianguation.
		int final_num_estimated_tracks = 0;

		// Number of triangulation attempts made.
		int num_triangulation_attempts = 0;

		// TrackId of the newly estimated tracks. This set does not include tracks
		// that were input as estimated.
		std::unordered_set<TrackId> estimated_tracks;

		double ba_setup_time_in_seconds = 0;
		double ba_total_time_in_seconds = 0;
	};

	CTrackEstimator(const Options& options, CReconstruction* reconstruction)
		: m_options(options), m_reconstruction(reconstruction) {}

	// Attempts to estimate all unestimated tracks.
	Summary EstimateAllTracks();

	// Estimate only the tracks supplied by the user.
	Summary EstimateTracks(const std::unordered_set<TrackId>& track_ids);

private:
	void EstimateTrackSet(const int start, const int stop);
	bool EstimateTrack(const TrackId track_id);

	const Options m_options;
	CReconstruction* m_reconstruction;
	std::vector<TrackId> m_tracks_to_estimate;
	std::unordered_set<TrackId> m_successfully_estimated_tracks;
};

#endif // __TRACK_ESTIMATOR_H__
