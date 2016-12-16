#include "TrackBuilder.h"

#include <glog/logging.h>

#include "Reconstruction.h"


CTrackBuilder::CTrackBuilder()
	:m_num_features(0)
{
}


CTrackBuilder::~CTrackBuilder()
{
}

void CTrackBuilder::AddFeatureCorrespondence(const ViewId view_id1,
	const Feature& feature1,
	const ViewId view_id2,
	const Feature& feature2) 
{
	CHECK_NE(view_id1, view_id2)
		<< "Cannot add 2 features from the same image as a correspondence for "
		"track generation.";

	const auto image_feature1 = std::make_pair(view_id1, feature1);
	const auto image_feature2 = std::make_pair(view_id2, feature2);

	const int feature1_id = FindOrInsert(image_feature1);
	const int feature2_id = FindOrInsert(image_feature2);

	m_connected_components->AddEdge(feature1_id, feature2_id);

}

void CTrackBuilder::BuildTracks(CReconstruction* reconstruction) {
	CHECK_NOTNULL(reconstruction);
	CHECK_EQ(reconstruction->NumTracks(), 0);

	// Build a reverse map mapping feature ids to ImageNameFeaturePairs.
	std::unordered_map<uint64_t, const std::pair<ViewId, Feature>*> id_to_feature;
	id_to_feature.reserve(m_features.size());
	for (const auto& feature : m_features) {
		InsertOrDie(&id_to_feature, feature.second, &feature.first);
	}

	// Extract all connected components.
	std::unordered_map<uint64_t, std::unordered_set<uint64_t> > components;
	m_connected_components->Extract(&components);

	// Each connected component is a track. Add all tracks to the reconstruction.
	int num_singleton_tracks = 0;
	int num_inconsistent_features = 0;
	for (const auto& component : components) {
		// Skip singleton tracks.
		if (component.second.size() == 1) {
			++num_singleton_tracks;
			continue;
		}

		std::vector<std::pair<ViewId, Feature> > track;
		track.reserve(component.second.size());

		// Add all features in the connected component to the track.
		std::unordered_set<ViewId> view_ids;
		for (const auto& feature_id : component.second) {
			const auto& feature_to_add = *FindOrDie(id_to_feature, feature_id);

			// Do not add the feature if the track already contains a feature from the
			// same image.
			if (!InsertIfNotPresent(&view_ids, feature_to_add.first)) {
				++num_inconsistent_features;
				continue;
			}

			track.emplace_back(feature_to_add);
		}

		CHECK_NE(reconstruction->AddTrack(track), kInvalidTrackId)
			<< "Could not build tracks.";
	}

	VLOG(1)
		<< reconstruction->NumTracks() << " tracks were created. "
		<< num_inconsistent_features
		<< " features were dropped because they formed inconsistent tracks, and "
		<< num_singleton_tracks
		<< " features were dropped because they formed singleton tracks.";
}

uint64_t CTrackBuilder::FindOrInsert(
	const std::pair<ViewId, Feature>& image_feature) {
	const uint64_t* feature_id = FindOrNull(m_features, image_feature);

	// If the feature is present, return the id.
	if (feature_id != nullptr) {
		return *feature_id;
	}

	// Otherwise, add the feature.
	const uint64_t new_feature_id = m_num_features;
	InsertOrDieNoPrint(&m_features, image_feature, new_feature_id);

	// Increment the number of features.
	++m_num_features;

	return new_feature_id;;
}
