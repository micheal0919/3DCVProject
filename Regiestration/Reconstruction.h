#ifndef __RECONSTRUCTION_H__
#define __RECONSTRUCTION_H__

#include <string>
#include <unordered_map>

#include "Types.h"

class CView;
class CTrack;

class CReconstruction {
public:
	CReconstruction();
	~CReconstruction();

	ViewId ViewIdFromName(const std::string& view_name) const;

	ViewId AddView(const std::string& view_name);

	bool RemoveView(const ViewId view_id);
	int NumViews() const;

	const CView* View(const ViewId view_id) const;
	CView* MutableView(const ViewId view_id);

	// Return all ViewIds in the reconstruction.
	std::vector<ViewId> ViewIds() const;

	TrackId AddTrack(const std::vector<std::pair<ViewId, Feature> >& track);

	bool RemoveTrack(const TrackId track_id);
	int NumTracks() const;

	const CTrack* Track(const TrackId track_id) const;
	CTrack* MutableTrack(const TrackId track_id);

	std::vector<TrackId> TrackIds() const;

	void Normalize();

private:
	TrackId m_next_track_id;
	ViewId m_next_view_id;

	std::unordered_map<std::string, ViewId> m_view_name_to_id;
	std::unordered_map<ViewId, CView> m_views;
	std::unordered_map<TrackId, CTrack> m_tracks;
};


#endif  // __RECONSTRUCTION_H__
