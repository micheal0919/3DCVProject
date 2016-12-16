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

	// Add all images to the reconstruction and get the camera project from file
	bool AddImages();

	// Extracts features and performs matching with geometric verification.
	bool ExtractAndMatchFeatures();

	// Estimates a Structure-from-Motion reconstruction using the specified
	// ReconstructionEstimator. Features are first extracted and matched if
	// necessary, then a reconstruction is estimated. Once a reconstruction has
	// been estimated, all views that have been successfully estimated are added
	// to the output vector and we estimate a reconstruction from the remaining
	// unestimated views. We repeat this process until no more views can be
	// successfully estimated.
	bool BuildReconstruction(std::vector<CReconstruction*>* reconstructions);

private:
//	CamaraProjection GetProjecttion(const std::string& image_path);
	
	// Add a match to the view graph. Either this method is repeatedly called or
	// ExtractAndMatchFeatures must be called.
	bool AddTwoViewMatch(const std::string& image1,
		const std::string& image2,
		const ImagePairMatch& matches);

	// Adds the given matches as edges in the view graph.
	void AddMatchToViewGraph(const ViewId view_id1,
		const ViewId view_id2,
		const ImagePairMatch& image_matches);

	// Builds tracks from the two view inlier correspondences after geometric
	// verification.
	void AddTracksForMatch(const ViewId view_id1,
		const ViewId view_id2,
		const ImagePairMatch& image_matches);


private:
	const ReconstructionBuilderOptions m_options;

	// Container of image information.
	std::vector<std::string> m_image_filepaths;

	// Container of image projection matrix
//	std::unordered_map<std::string, CamaraProjection> m_camera_projections;
//	std::map<std::string, CamaraProjection> m_camera_projections;

	// Module for performing feature extraction and matching.
	std::unique_ptr<CFeatureExtractorAndMatcher> m_feature_extractor_and_matcher;

	std::unique_ptr<CReconstruction> m_reconstruction;
	std::unique_ptr<CTrackBuilder> m_track_builder;
	std::unique_ptr<CViewGraph> m_view_graph;

};

#endif // __RECONSTRUCTION_BUILDER_H__
