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
		double max_acceptable_reprojection_error_pixels = 5.0;
		double min_triangulation_angle_degrees = 3.0;

		bool bundle_adjustment = false;
		BundleAdjustmentOptions ba_options;
	};

	struct Summary {
		int input_num_estimated_tracks = 0;
		int final_num_estimated_tracks = 0;
		int num_triangulation_attempts = 0;
		std::unordered_set<TrackId> estimated_tracks;
		double ba_setup_time_in_seconds = 0;
		double ba_total_time_in_seconds = 0;
	};

	CTrackEstimator(const Options& options, CReconstruction* reconstruction)
		: m_options(options), m_reconstruction(reconstruction) {}

	Summary EstimateAllTracks();
	Summary EstimateTracks(const std::unordered_set<TrackId>& track_ids);

private:
	void EstimateTrackSet(const int start, const int stop);
	bool EstimateTrack(const TrackId track_id);
	
	// estimate tracks using TriangulateMidpoint algorithm
	bool EstimateTrackMidpoint(const TrackId track_id);
	
	// estimate tracks using TriangulateNViewSVD algorithm
	bool EstimateTrackNViewSVD(const TrackId track_id);

	// estimate tracks using TriangulateNView algorithm
	bool EstimateTrackNView(const TrackId track_id);
	
	void reconstruction_test();

	const Options m_options;
	CReconstruction* m_reconstruction;
	std::vector<TrackId> m_tracks_to_estimate;
	std::unordered_set<TrackId> m_successfully_estimated_tracks;
};

#endif // __TRACK_ESTIMATOR_H__
