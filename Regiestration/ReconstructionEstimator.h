#ifndef __RECONSTRUCTION_ESTIMATOR_H__
#define __RECONSTRUCTION_ESTIMATOR_H__

#include <unordered_set>
#include <unordered_map>

#include "Types.h"
#include "util/uitl.h"

class CViewGraph;
class CReconstruction;

struct ReconstructionEstimatorSummary 
{
	bool success = false;
	std::unordered_set<ViewId> estimated_views;
	std::unordered_set<TrackId> estimated_tracks;

	double camera_intrinsics_calibration_time = 0.0;
	double pose_estimation_time = 0.0;
	double triangulation_time = 0.0;
	double bundle_adjustment_time = 0.0;
	double total_time = 0.0;

	std::string message;
};

enum class ReconstructionEstimatorType 
{
	GLOBAL = 0,
	INCREMENTAL = 1
};

enum class GlobalRotationEstimatorType 
{
	ROBUST_L1L2 = 0,
	NONLINEAR = 1,
	LINEAR = 2
};


enum class GlobalPositionEstimatorType 
{
	NONLINEAR = 0,
	LINEAR_TRIPLET = 1,
	LEAST_UNSQUARED_DEVIATION = 2,
};


struct ReconstructionEstimatorOptions 
{
	// Type of reconstruction estimation to use.
	ReconstructionEstimatorType reconstruction_estimator_type =
		ReconstructionEstimatorType::GLOBAL;

	// If Global SfM is desired, which type of rotation and position estimation
	// methods are used.
	GlobalRotationEstimatorType global_rotation_estimator_type =
		GlobalRotationEstimatorType::ROBUST_L1L2;

	GlobalPositionEstimatorType global_position_estimator_type =
		GlobalPositionEstimatorType::NONLINEAR;

	// Number of threads to use.
	int num_threads = 1;

	// Maximum reprojection error. This is the threshold used for filtering
	// outliers after bundle adjustment.
	double max_reprojection_error_in_pixels = 5.0;

	// Any edges in the view graph with fewer than min_num_two_view_inliers will
	// be removed as an initial filtering step.
	int min_num_two_view_inliers = 30;

	// After computing a model and performing an initial BA, the reconstruction
	// can be further improved (and even densified) if we attempt (again) to
	// retriangulate any tracks that are currently unestimated. For each
	// retriangulation iteration we do the following:
	//   1. Remove features that are above max_reprojection_error_in_pixels.
	//   2. Triangulate all unestimated tracks.
	//   3. Perform full bundle adjustment.
	int num_retriangulation_iterations = 1;

	// --------------- RANSAC Options --------------- //
	double ransac_confidence = 0.9999;
	int ransac_min_iterations = 50;
	int ransac_max_iterations = 1000;
	bool ransac_use_mle = true;

	// --------------- Rotation Filtering Options --------------- //

	// After orientations are estimated, view pairs may be filtered/removed if the
	// relative rotation of the view pair differs from the relative rotation
	// formed by the global orientation estimations. Adjust this threshold to
	// control the threshold at which rotations are filtered. See
	// theia/sfm/filter_view_pairs_from_orientation.h
	double rotation_filtering_max_difference_degrees = 5.0;

	// --------------- Position Filtering Options --------------- //

	// Refine the relative translations based on the epipolar error and known
	// rotation estimations. This improve the quality of the translation
	// estimation.
	bool refine_relative_translations_after_rotation_estimation = true;

	// If true, the maximal rigid component of the viewing graph will be
	// extracted. This means that only the cameras that are well-constrained for
	// position estimation will be used. This method is somewhat slow, so enabling
	// it will cause a performance hit in terms of efficiency.
	//
	// NOTE: This method does not attempt to remove outlier 2-view geometries, it
	// only determines which cameras are well-conditioned for position estimation.
	bool extract_maximal_rigid_subgraph = false;

	// If true, filter the pairwise translation estimates to remove potentially
	// bad relative poses. Removing potential outliers can increase the
	// performance of position estimation.
	bool filter_relative_translations_with_1dsfm = true;

	// Before the camera positions are estimated, it is wise to remove any
	// relative translations estimates that are low quality. See
	// theia/sfm/filter_view_pairs_from_relative_translation.h
	int translation_filtering_num_iterations = 48;
	double translation_filtering_projection_tolerance = 0.1;

	// --------------- Global Rotation Estimation Options --------------- //

	// Robust loss function scales for nonlinear estimation.
	double rotation_estimation_robust_loss_scale = 0.1;

	// --------------- Global Position Estimation Options --------------- //
	//NonlinearPositionEstimator::Options nonlinear_position_estimator_options;
	//LinearPositionEstimator::Options linear_triplet_position_estimator_options;
	//LeastUnsquaredDeviationPositionEstimator::Options
	//	least_unsquared_deviation_position_estimator_options;

	// --------------------- Incremental SfM Options --------------------- //

	// If M is the maximum number of 3D points observed by any view, we want to
	// localize all views that observe > M * multiple_view_localization_ratio 3D
	// points. This allows for multiple well-conditioned views to be added to the
	// reconstruction before needing bundle adjustment.
	double multiple_view_localization_ratio = 0.8;

	// When adding a new view to the current reconstruction, this is the
	// reprojection error that determines whether a 2D-3D correspondence is an
	// inlier during localization.
	double absolute_pose_reprojection_error_threshold = 8.0;

	// Minimum number of inliers for absolute pose estimation to be considered
	// successful.
	int min_num_absolute_pose_inliers = 30;

	// Bundle adjustment of the entire reconstruction is triggered when the
	// reconstruction has grown by more than this percent. That is, if we last ran
	// BA when there were K views in the reconstruction and there are now N views,
	// then G = (N - K) / K is the percent that the model has grown. We run bundle
	// adjustment only if G is greater than this variable. This variable is
	// indicated in percent so e.g., 5.0 = 5%.
	double full_bundle_adjustment_growth_percent = 5.0;

	// During incremental SfM we run "partial" bundle adjustment on the most
	// recent views that have been added to the 3D reconstruction. This parameter
	// controls how many views should be part of the partial BA.
	int partial_bundle_adjustment_num_views = 20;

	// --------------- Triangulation Options --------------- //

	// Minimum angle required between a 3D point and 2 viewing rays in order to
	// consider triangulation a success.
	double min_triangulation_angle_degrees = 3.0;

	// The reprojection error to use for determining valid triangulation.
	double triangulation_max_reprojection_error_in_pixels = 10.0;
	// Bundle adjust a track immediately after estimating it.
	bool bundle_adjust_tracks = true;

	// --------------- Bundle Adjustment Options --------------- //

	// For bundle adjustment, we may want to use a robust loss function to improve
	// robustness to outliers. The various types of robust loss functions used can
	// be found at //theia/sfm/bundle_adjustment/create_loss_function.h
	//LossFunctionType bundle_adjustment_loss_function_type =
	//	LossFunctionType::TRIVIAL;

	// For robust loss functions, the robustness will begin for values that have
	// an error greater than this value. For example, Tukey loss will have a
	// constant loss when the error values are greater than this.
	double bundle_adjustment_robust_loss_width = 10.0;

	// Use SPARSE_SCHUR for problems smaller than this size and ITERATIVE_SCHUR
	// for problems larger than this size.
	int min_cameras_for_iterative_solver = 1000;

	// If accurate calibration is known ahead of time then it is recommended to
	// set the camera intrinsics constant during bundle adjustment. Othewise, you
	// can choose which intrinsics to optimize. See
	// //theia/sfm/bundle_adjustment_options.h for full details.
	//OptimizeIntrinsicsType intrinsics_to_optimize =
	//	OptimizeIntrinsicsType::FOCAL_LENGTH |
	//	OptimizeIntrinsicsType::PRINCIPAL_POINTS |
	//	OptimizeIntrinsicsType::RADIAL_DISTORTION;
};

// A reconstruction estimator should build a reconstruction from a view graph
// and an unestimated reconstruction with the corresponding views of the view
// graph. The camera poses and 3D point positions are estimated with this
// class. The focus of the Theia library is to create global methods for SfM
// such that all camera pose are estimated simultaneously then points are
// estimated afterwards.
class CReconstructionEstimator {
public:
	virtual ~CReconstructionEstimator() {}

	// Estimates the camera poses for a reconstruction given the view graph
	// describing the multi-view correspondences.
	virtual ReconstructionEstimatorSummary Estimate(
		CViewGraph* view_graph, CReconstruction* reconstruction) = 0;

	static CReconstructionEstimator* Create(
		const ReconstructionEstimatorOptions& options);
};

// Estimates the camera position and 3D structure of the scene using an
// incremental Structure from Motion approach. The method begins by first
// estimating the 3D structure and camera poses of 2 cameras based on their
// relative pose. Then additional cameras are added on sequentially and new 3D
// structure is estimated as new parts of the scene are observed. Bundle
// adjustment is repeatedly performed as more cameras are added to ensure high
// quality reconstructions and to avoid drift.
//
// The incremental SfM pipeline is as follows:
//   1) Choose an initial camera pair to reconstruct.
//   2) Estimate 3D structure of the scene.
//   3) Bundle adjustment on the 2-view reconstruction.
//   4) Localize a new camera to the current 3D points. Choose the camera that
//      observes the most 3D points currently in the scene.
//   5) Estimate new 3D structure.
//   6) Bundle adjustment if the model has grown by more than 5% since the last
//      bundle adjustment.
//   7) Repeat steps 4-6 until all cameras have been added.
//
// Incremental SfM is generally considered to be more robust than global SfM
// methods; hwoever, it requires many more instances of bundle adjustment (which
// is very costly) and so incremental SfM is not as efficient or scalable.
class CIncrementalReconstructionEstimator : public CReconstructionEstimator {
public:
	CIncrementalReconstructionEstimator(
		const ReconstructionEstimatorOptions& options);

	ReconstructionEstimatorSummary Estimate(CViewGraph* view_graph,
		CReconstruction* reconstruction);

private:
	DISALLOW_COPY_AND_ASSIGN(CIncrementalReconstructionEstimator);
};


// Estimates the camera position and 3D structure of the scene using global
// methods to estimate camera poses. First, rotation is estimated globally
// then the position is estimated using a global optimization.
//
// The pipeline for estimating camera poses and structure is as follows:
//   1) Filter potentially bad pairwise geometries by enforcing a loop
//      constaint on rotations that form a triplet.
//   2) Initialize focal lengths.
//   3) Estimate the global rotation for each camera.
//   4) Remove any pairwise geometries where the relative rotation is not
//      consistent with the global rotation.
//   5) Optimize the relative translation given the known rotations.
//   6) Filter potentially bad relative translations.
//   7) Estimate positions.
//   8) Estimate structure.
//   9) Bundle adjustment.
//   10) Retriangulate, and bundle adjust.
//
// After each filtering step we remove any views which are no longer connected
// to the largest connected component in the view graph.
class CGlobalReconstructionEstimator : public CReconstructionEstimator {
public:
	CGlobalReconstructionEstimator(
		const ReconstructionEstimatorOptions& options);

	ReconstructionEstimatorSummary Estimate(CViewGraph* view_graph,
		CReconstruction* reconstruction);

private:
	bool FilterInitialViewGraph();
	void CalibrateCameras();
	bool EstimateGlobalRotations();
	void FilterRotations();
	void OptimizePairwiseTranslations();
	void FilterRelativeTranslation();
	bool EstimatePosition();
	void EstimateStructure();
	bool BundleAdjustment();

private:
	CViewGraph* m_view_graph;
	CReconstruction* m_reconstruction;

	ReconstructionEstimatorOptions m_options;

//	FilterViewPairsFromRelativeTranslationOptions translation_filter_options_;
	
	BundleAdjustmentOptions m_bundle_adjustment_options;

//	RansacParameters ransac_params_;

	std::unordered_map<ViewId, Eigen::Vector3d> m_orientations;
	std::unordered_map<ViewId, Eigen::Vector3d> m_positions;

	DISALLOW_COPY_AND_ASSIGN(CGlobalReconstructionEstimator);
};

#endif // __RECONSTRUCTION_ESTIMATOR_H__
