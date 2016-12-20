#include "Reconstruction.h"

#include <glog/logging.h>

#include "MapUtils.h"
#include "View.h"
#include "Track.h"


namespace {

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

ViewId CReconstruction::ViewIdFromName(const std::string& view_name) const
{
	return FindWithDefault(m_view_name_to_id, view_name, kInvalidViewId);
}

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

		if (track->NumViews() == 0) {
			RemoveTrack(track_id);
		}
	}

	const std::string& view_name = view->Name();
	m_view_name_to_id.erase(view_name);

	m_views.erase(view_id);
	return true;
}
int CReconstruction::NumViews() const
{
	return m_views.size();
}

const CView* CReconstruction::View(const ViewId view_id) const
{
	return FindOrNull(m_views, view_id);
}

CView* CReconstruction::MutableView(const ViewId view_id)
{
	return FindOrNull(m_views, view_id);
}

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
		CHECK(ContainsKey(m_views, observation.first))
			<< "Cannot add a track with containing an observation in view id "
			<< observation.first << " because the view does not exist.";
		new_track.AddView(observation.first);
		CView* view = MutableView(observation.first);
		view->AddFeature(new_track_id, observation.second);
	}

	m_tracks.emplace(new_track_id, new_track);
	++m_next_track_id;
	return new_track_id;
}

bool CReconstruction::RemoveTrack(const TrackId track_id)
{
	CTrack* track = FindOrNull(m_tracks, track_id);
	if (track == nullptr) {
		LOG(WARNING) << "Cannot remove a track that does not exist";
		return false;
	}

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

	m_tracks.erase(track_id);
	return true;
}

int CReconstruction::NumTracks() const
{
	return m_tracks.size();
}

const CTrack* CReconstruction::Track(const TrackId track_id) const
{
	return FindOrNull(m_tracks, track_id);
}

CTrack* CReconstruction::MutableTrack(const TrackId track_id)
{
	return FindOrNull(m_tracks, track_id);
}

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

