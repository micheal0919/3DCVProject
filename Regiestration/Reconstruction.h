#ifndef __RECONSTRUCTION_H__
#define __RECONSTRUCTION_H__

#include <string>
#include <unordered_map>

#include "Types.h"

class CView;
class CTrack;

// A Reconstruction is the main object for structure from motion. It contains
// all the 3D and camera information as well as visibility constraints. That is,
// it contains Tracks and Views and the constraints between them.

class CReconstruction {
public:
	CReconstruction();
	~CReconstruction();

	// Returns the unique ViewId of the view name, or kInvalidViewId if the view
	// does not
	// exist.
	ViewId ViewIdFromName(const std::string& view_name) const;

	// Creates a new view and returns the view id. If the view
	ViewId AddView(const std::string& view_name);

	// Removes the view from the reconstruction and removes all references to the
	// view in the tracks. Any tracks that have zero views after this view is
	// removed are alsoremoved.
	bool RemoveView(const ViewId view_id);
	int NumViews() const;

	// Returns the View or a nullptr if the track does not exist.
	const CView* View(const ViewId view_id) const;
	CView* MutableView(const ViewId view_id);

	// Return all ViewIds in the reconstruction.
	std::vector<ViewId> ViewIds() const;

	// Add a new track to the reconstruction. If successful, the new track id is
	// returned. Failure results when multiple features from the same image are
	// present, and kInvalidTrackId is returned.
	TrackId AddTrack(const std::vector<std::pair<ViewId, Feature> >& track);

	// Removes the track from the reconstruction including the corresponding
	// features that are present in the view that observe it.
	bool RemoveTrack(const TrackId track_id);
	int NumTracks() const;

	// Returns the Track or a nullptr if the track does not exist.
	const CTrack* Track(const TrackId track_id) const;
	CTrack* MutableTrack(const TrackId track_id);

	// Return all TrackIds in the reconstruction.
	std::vector<TrackId> TrackIds() const;

	// Normalizes the reconstruction such that the "center" of the reconstruction
	// is moved to the origin and the reconstruction is scaled such that the
	// median distance of 3D points from the origin is 100.0. This does not affect
	// the reprojection error. A rotation is applied such that the x-z plane is
	// set to the dominating plane of the cameras.
	//
	// NOTE: This implementation is inspired by the BAL problem normalization in
	// Ceres Solver.
	void Normalize();

private:
	TrackId m_next_track_id;
	ViewId m_next_view_id;

	std::unordered_map<std::string, ViewId> m_view_name_to_id;
	std::unordered_map<ViewId, CView> m_views;
	std::unordered_map<TrackId, CTrack> m_tracks;
};


#endif  // __RECONSTRUCTION_H__
