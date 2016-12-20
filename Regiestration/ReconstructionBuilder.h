#ifndef __RECONSTRUCTION_BUILDER_H__
#define __RECONSTRUCTION_BUILDER_H__

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

#include "Types.h"
#include "FeatureExtractorAndMatcher.h"
#include "ReconstructionEstimator.h"

class CReconstruction;
class CTrackBuilder;
class CViewGraph;

struct ReconstructionBuilderOptions 
{
	// input images options
	std::string image_path;
	std::string image_info_file_path;
	int num_image;

	// Set to true to only accept calibrated views (from EXIF or elsewhere) as
	// valid inputs to the reconstruction process. When uncalibrated views are
	// added to the reconstruction builder they are ignored with a LOG warning.
	bool only_calibrated_views = false;

	// Maximum allowable track length. Tracks that are too long are exceedingly
	// likely to contain outliers.
	int max_track_length = 20;

	// Type of reconstruction estimation to use.
	ReconstructionEstimatorType reconstruction_estimator_type =
		ReconstructionEstimatorType::GLOBAL;

	// 	Feature Extract And Match Options
	CFeatureExtractorAndMatcher::Options feam_options;

	// Options for estimating the reconstruction.
	// See ReconstructionEstimator.h
	ReconstructionEstimatorOptions reconstruction_estimator_options;


};

class CReconstructionBuilder
{

public:
	explicit CReconstructionBuilder(const ReconstructionBuilderOptions& options);
	~CReconstructionBuilder();

	bool AddImages();
	bool ExtractAndMatchFeatures();
	bool BuildReconstruction(std::vector<CReconstruction*>* reconstructions);

private:
//	CamaraProjection GetProjecttion(const std::string& image_path);
	
	bool AddTwoViewMatch(const std::string& image1,
		const std::string& image2,
		const ImagePairMatch& matches);

	void AddMatchToViewGraph(const ViewId view_id1,
		const ViewId view_id2,
		const ImagePairMatch& image_matches);

	void AddTracksForMatch(const ViewId view_id1,
		const ViewId view_id2,
		const ImagePairMatch& image_matches);


private:
	const ReconstructionBuilderOptions m_options;
	std::vector<std::string> m_image_filepaths;
	std::unique_ptr<CFeatureExtractorAndMatcher> m_feature_extractor_and_matcher;
	std::unique_ptr<CReconstruction> m_reconstruction;
	std::unique_ptr<CTrackBuilder> m_track_builder;
	std::unique_ptr<CViewGraph> m_view_graph;

};

#endif // __RECONSTRUCTION_BUILDER_H__
