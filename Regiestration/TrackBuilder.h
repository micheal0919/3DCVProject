#ifndef __TRACK_BUILDER_H__
#define __TRACK_BUILDER_H__

#include <stdint.h>
#include <memory>
#include <unordered_map>
#include <functional>

#include "Types.h"
#include "ConnectedComponents.h"

class CReconstruction;

class CTrackBuilder
{
public:
	explicit CTrackBuilder(const int max_track_length);
	~CTrackBuilder();

	// Adds a feature correspondence between two views.
	void AddFeatureCorrespondence(const ViewId view_id1, const Feature& feature1,
		const ViewId view_id2, const Feature& feature2);

	// Generates all tracks and adds them to the reconstruction.
	void BuildTracks(CReconstruction* reconstruction);

private:
	uint64_t FindOrInsert(const std::pair<ViewId, Feature>& image_feature);

private:
	// hash fucntion for unordered_map
	struct pair_hash
	{
		std::size_t operator() (const std::pair<ViewId, Feature>& p) const
		{
			auto h1 = std::hash<ViewId>{}(p.first);
			auto h2 = std::hash<double>{}(p.second.x);
			auto h3 = std::hash<double>{}(p.second.y);

			return h1 ^ h2 ^ h3;
		}

	};

	std::unordered_map<std::pair<ViewId, Feature>, uint64_t, pair_hash> m_features;
	std::unique_ptr<CConnectedComponents<uint64_t> > m_connected_components;
	uint64_t m_num_features;
};

#endif // __TRACK_BUILDER_H__
