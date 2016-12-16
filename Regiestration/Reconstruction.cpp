#include "Reconstruction.h"

#include <glog/logging.h>

#include "MapUtils.h"
#include "View.h"
#include "Track.h"


namespace {

	// Return whether the track contains the same view twice.
	bool DuplicateViewsExistInTrack(
		const std::vector<std::pair<ViewId, Feature> >& track) {
		std::vector<ViewId> view_ids;
		view_ids.reserve(track.size());
		for (const auto& feature : track) {
			view_ids.push_back(feature.first);
		}
		std::sort(view_ids.begin(), view_ids.end());
		return (std::adjacent_find(view_ids.begin(), view_ids.end()) !=
			view_ids.end());
	}

	double Median(std::vector<double>* data) {
		int n = data->size();
		std::vector<double>::iterator mid_point = data->begin() + n / 2;
		std::nth_element(data->begin(), mid_point, data->end());
		return *mid_point;
	}

}  // namespace

CReconstruction::CReconstruction() : m_next_track_id(0), m_next_view_id(0) {}

CReconstruction::~CReconstruction() {}

// Returns the unique ViewId of the view name, or kInvalidViewId if the view
// does not
// exist.
ViewId CReconstruction::ViewIdFromName(const std::string& view_name) const
{
	return FindWithDefault(m_view_name_to_id, view_name, kInvalidViewId);
}

// Creates a new view and returns the view id. If the view
ViewId CReconstruction::AddView(const std::string& view_name)
{
	if (ContainsKey(m_view_name_to_id, view_name)) {
		LOG(WARNING) << "Could not add view with the name " << view_name
			<< " because that name already exists in the reconstruction.";
		return kInvalidViewId;
	}

	if (view_name.empty()) {
		LOG(WARNING)
			<< "View name was empty. Could not add view to reconstruction.";
		return kInvalidViewId;
	}

	CView new_view(view_name);
	m_views.emplace(m_next_view_id, new_view);
	m_view_name_to_id.emplace(view_name, m_next_view_id);

	++m_next_view_id;
	return m_next_view_id - 1;
}

// Removes the view from the reconstruction and removes all references to the
// view in the tracks. Any tracks that have zero views after this view is
// removed are alsoremoved.
bool CReconstruction::RemoveView(const ViewId view_id)
{
	CView* view = FindOrNull(m_views, view_id);
	if (view == nullptr) {
		LOG(WARNING)
			<< "Could not remove the view from the reconstruction because the view "
			"does not exist.";
		return false;
	}

	const auto& track_ids = view->TrackIds();
	for (const TrackId track_id : track_ids) {
		CTrack* track = MutableTrack(track_id);
		if (track == nullptr) {
			LOG(WARNING) << "Could not remove the view from the track because the "
				"track does not exist";
			return false;
		}

		if (!track->RemoveView(view_id)) {
			LOG(WARNING) << "Could not remove to view from the track";
			return false;
		}

		// Remove the track if it no longer contains any views.
		if (track->NumViews() == 0) {
			RemoveTrack(track_id);
		}
	}

	// Remove the view name.
	const std::string& view_name = view->Name();
	m_view_name_to_id.erase(view_name);

	// Remove the view.
	m_views.erase(view_id);
	return true;
}
int CReconstruction::NumViews() const
{
	return m_views.size();
}

// Returns the View or a nullptr if the track does not exist.
const CView* CReconstruction::View(const ViewId view_id) const
{
	return FindOrNull(m_views, view_id);
}

CView* CReconstruction::MutableView(const ViewId view_id)
{
	return FindOrNull(m_views, view_id);
}

// Return all ViewIds in the reconstruction.
std::vector<ViewId> CReconstruction::ViewIds() const
{
	std::vector<ViewId> view_ids;
	view_ids.reserve(m_views.size());
	for (const auto& view : m_views)
	{
		view_ids.push_back(view.first);
	}
	return view_ids;
}

// Add a new track to the reconstruction. If successful, the new track id is
// returned. Failure results when multiple features from the same image are
// present, and kInvalidTrackId is returned.
TrackId CReconstruction::AddTrack(const std::vector<std::pair<ViewId, Feature> >& track)
{
	if (track.size() < 2) {
		LOG(WARNING) << "Tracks must have at least 2 observations (" << track.size()
			<< " were given). Cannot add track to the reconstruction";
		return kInvalidTrackId;
	}

	if (DuplicateViewsExistInTrack(track)) {
		LOG(WARNING) << "Cannot add a track that contains the same view twice to "
			"the reconstruction.";
		return kInvalidTrackId;
	}

	const TrackId new_track_id = m_next_track_id;
	CHECK(!ContainsKey(m_tracks, new_track_id))
		<< "The reconstruction already contains a track with id: "
		<< new_track_id;

	CTrack new_track;

	std::vector<ViewId> views_to_add;
	for (const auto& observation : track) {
		// Make sure the view exists in the model.
		CHECK(ContainsKey(m_views, observation.first))
			<< "Cannot add a track with containing an observation in view id "
			<< observation.first << " because the view does not exist.";

		// Add view to track.
		new_track.AddView(observation.first);

		// Add track to view.
		CView* view = MutableView(observation.first);
		view->AddFeature(new_track_id, observation.second);
	}

	m_tracks.emplace(new_track_id, new_track);
	++m_next_track_id;
	return new_track_id;
}

// Removes the track from the reconstruction including the corresponding
// features that are present in the view that observe it.
bool CReconstruction::RemoveTrack(const TrackId track_id)
{
	CTrack* track = FindOrNull(m_tracks, track_id);
	if (track == nullptr) {
		LOG(WARNING) << "Cannot remove a track that does not exist";
		return false;
	}

	// Remove track from views.
	for (const ViewId view_id : track->ViewIds()) {
		CView* view = FindOrNull(m_views, view_id);
		if (view == nullptr) {
			LOG(WARNING) << "Could not remove a track from the view because the view "
				"does not exist";
			return false;
		}

		if (!view->RemoveFeature(track_id)) {
			LOG(WARNING) << "Could not remove the track from the view because the "
				"track is not observed by the view.";
			return false;
		}
	}

	// Delete from the reconstruction.
	m_tracks.erase(track_id);
	return true;
}

int CReconstruction::NumTracks() const
{
	return m_tracks.size();
}

// Returns the Track or a nullptr if the track does not exist.
const CTrack* CReconstruction::Track(const TrackId track_id) const
{
	return FindOrNull(m_tracks, track_id);
}

CTrack* CReconstruction::MutableTrack(const TrackId track_id)
{
	return FindOrNull(m_tracks, track_id);
}

// Return all TrackIds in the reconstruction.
std::vector<TrackId> CReconstruction::TrackIds() const
{
	std::vector<TrackId> track_ids;
	track_ids.reserve(m_tracks.size());
	for (const auto& track : m_tracks)
	{
		track_ids.push_back(track.first);
	}
	return track_ids;
}

