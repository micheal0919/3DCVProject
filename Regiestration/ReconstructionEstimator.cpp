#include "ReconstructionEstimator.h"

#include <iostream>

#include <glog/logging.h>
#include <Eigen/Core>
#include <Eigen/Dense>

#include "Camera.h"
#include "View.h"
#include "ViewGraph.h"
#include "Reconstruction.h"
#include "Track.h"
#include "TrackEstimator.h"
#include "util/timer.h"

namespace
{
	// All times are given in seconds.
	struct GlobalReconstructionEstimatorTimings {
		double initial_view_graph_filtering_time = 0.0;
		double camera_intrinsics_calibration_time = 0.0;
		double rotation_estimation_time = 0.0;
		double rotation_filtering_time = 0.0;
		double relative_translation_optimization_time = 0.0;
		double relative_translation_filtering_time = 0.0;
		double position_estimation_time = 0.0;
	};


	// Sets the camera intrinsics from the CameraIntrinsicsPrior of a view. If the
	// view does not have a focal length prior will set a value corresponding to a
	// median viewing angle. Principal points that are not provided by the priors
	// are simply initialized as half of the corresponding image size dimension.
	void SetViewCameraIntrinsicsFromPriors(CView* view)
	{
		LOG(INFO);

		CCamera* camera = view->MutableCamera();
		const CameraIntrinsicsPrior prior = view->CameraIntrinsicsPrior();

		// Set the image dimensions.
		camera->SetImageSize(prior.image_width, prior.image_height);

		// Set the focal length.
		if (prior.focal_length.is_set) {
			LOG(INFO);
			camera->SetFocalLength(prior.focal_length.value);
		}
		else {
			LOG(INFO);
			camera->SetFocalLength(1.2 * static_cast<double>(prior.image_width > prior.image_height ? prior.image_width : prior.image_height));
		}

		// Set the principal point.
		if (prior.principal_point[0].is_set && prior.principal_point[1].is_set) {
			LOG(INFO);
			camera->SetPrincipalPoint(prior.principal_point[0].value,
				prior.principal_point[1].value);
		}
		else {
			LOG(INFO);
			camera->SetPrincipalPoint(prior.image_width / 2.0,
				prior.image_height / 2.0);
		}

		// Set aspect ratio if available.
		if (prior.aspect_ratio.is_set) {
			LOG(INFO);
			camera->SetAspectRatio(prior.aspect_ratio.value);
		}

		// Set skew if available.
		if (prior.skew.is_set) {
			LOG(INFO);
			camera->SetSkew(prior.skew.value);
		}

		// Set radial distortion if available.
		if (prior.radial_distortion[0].is_set &&
			prior.radial_distortion[1].is_set) {
			LOG(INFO);
			camera->SetRadialDistortion(prior.radial_distortion[0].value,
				prior.radial_distortion[1].value);
		}

		LOG(INFO);
	}

	// Sets the camera intrinsics from the CameraIntrinsicsPrior of each view. Views
	// that do not have a focal length prior will set a value corresponding to a
	// median viewing angle. Principal points that are not provided by the priors
	// are simply initialized as half of the corresponding image size dimension.
	void SetCameraIntrinsicsFromPriors(CReconstruction* reconstruction) 
	{
		LOG(INFO);

		const auto& view_ids = reconstruction->ViewIds();
		for (const ViewId view_id : view_ids) {
			CView* view = CHECK_NOTNULL(reconstruction->MutableView(view_id));
			SetViewCameraIntrinsicsFromPriors(view);
		}

		LOG(INFO);
	}

	// Removes features that have a reprojection error larger than the
	// reprojection error threshold. Additionally, any features that are poorly
	// constrained because of a small viewing angle are removed. Returns the number
	// of features removed. Only the input tracks are checked.
	int RemoveOutlierFeatures(const std::unordered_set<TrackId>& track_ids,
		const double max_inlier_reprojection_error,
		const double min_triangulation_angle_degrees,
		CReconstruction* reconstruction) 
	{
		return 0;
	}

	// Removes features that have a reprojection error larger than the
	// reprojection error threshold. Additionally, any features that are poorly
	// constrained because of a small viewing angle are removed. Returns the number
	// of features removed. Only the input tracks are checked.
	int RemoveOutlierFeatures(const double max_inlier_reprojection_error,
		const double min_triangulation_angle_degrees,
		CReconstruction* reconstruction) 
	{
		const auto& track_ids = reconstruction->TrackIds();
		const std::unordered_set<TrackId> all_tracks(track_ids.begin(),
			track_ids.end());
		return RemoveOutlierFeatures(all_tracks,
			max_inlier_reprojection_error,
			min_triangulation_angle_degrees,
			reconstruction);
	}

	// Outputs the ViewId of all estimated views in the reconstruction.
	void GetEstimatedViewsFromReconstruction(const CReconstruction& reconstruction,
		std::unordered_set<ViewId>* views) 
	{
		CHECK_NOTNULL(views)->clear();
		for (const ViewId view_id : reconstruction.ViewIds()) {
			const CView* view = reconstruction.View(view_id);
			if (view != nullptr && view->IsEstimated()) {
				views->insert(view_id);
			}
		}
	}

	// Outputs the TrackId of all estimated tracks in the reconstruction.
	void GetEstimatedTracksFromReconstruction(const CReconstruction& reconstruction,
		std::unordered_set<TrackId>* tracks) 
	{
		CHECK_NOTNULL(tracks)->clear();
		const auto& track_ids = reconstruction.TrackIds();
		for (const TrackId track_id : track_ids) {
			const CTrack* track = reconstruction.Track(track_id);
			if (track != nullptr && track->IsEstimated()) {
				tracks->insert(track_id);
			}
		}
	}

}

CReconstructionEstimator* CReconstructionEstimator::Create(
	const ReconstructionEstimatorOptions& options) 
{
	LOG(INFO);

	switch (options.reconstruction_estimator_type) {
	case ReconstructionEstimatorType::GLOBAL:
		LOG(INFO);
		return new CGlobalReconstructionEstimator(options);
		break;
	case ReconstructionEstimatorType::INCREMENTAL:
		LOG(INFO);
		return new CIncrementalReconstructionEstimator(options);
		break;
	default:
		LOG(FATAL) << "Invalid reconstruction estimator specified.";
	}
	return nullptr;
}

CIncrementalReconstructionEstimator::CIncrementalReconstructionEstimator(const ReconstructionEstimatorOptions& options) 
{
	LOG(INFO);
}

ReconstructionEstimatorSummary CIncrementalReconstructionEstimator::Estimate(CViewGraph* view_graph, CReconstruction* reconstruction)
{
	LOG(INFO);
	ReconstructionEstimatorSummary summary;
	return summary;
}

CGlobalReconstructionEstimator::CGlobalReconstructionEstimator(const ReconstructionEstimatorOptions& options) 
{
	LOG(INFO);
	m_options = options;
}

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
ReconstructionEstimatorSummary CGlobalReconstructionEstimator::Estimate(CViewGraph* view_graph, CReconstruction* reconstruction) 
{
	LOG(INFO) << "Beginning of CGlobalReconstructionEstimator::Estimate";

	CHECK_NOTNULL(reconstruction);
	m_reconstruction = reconstruction;
	m_view_graph = view_graph;
	m_orientations.clear();
	m_positions.clear();

	LOG(INFO);

	ReconstructionEstimatorSummary summary;
	GlobalReconstructionEstimatorTimings global_estimator_timings;
	theia::Timer total_timer;
	theia::Timer timer;

	LOG(INFO);

	// Step 1. Filter the initial view graph and remove any bad two view
	// geometries.
	LOG(INFO) << "Filtering the intial view graph.";
	timer.Reset();
	if (!FilterInitialViewGraph()) {
		LOG(INFO) << "Insufficient view pairs to perform estimation.";
		return summary;
	}
	global_estimator_timings.initial_view_graph_filtering_time =
		timer.ElapsedTimeInSeconds();

	// Step 2. Calibrate any uncalibrated cameras.
	LOG(INFO) << "Calibrating any uncalibrated cameras.";
	timer.Reset();
	CalibrateCameras();
	summary.camera_intrinsics_calibration_time = timer.ElapsedTimeInSeconds();

	// Step 3. Estimate global rotations.
	LOG(INFO) << "Estimating the global rotations of all cameras.";
	timer.Reset();
	if (!EstimateGlobalRotations()) {
		LOG(WARNING) << "Rotation estimation failed!";
		summary.success = false;
		return summary;
	}
	global_estimator_timings.rotation_estimation_time =
		timer.ElapsedTimeInSeconds();

	// Step 4. Filter bad rotations.
	LOG(INFO) << "Filtering any bad rotation estimations.";
	timer.Reset();
	FilterRotations();
	global_estimator_timings.rotation_filtering_time =
		timer.ElapsedTimeInSeconds();

	// Step 5. Optimize relative translations.
	LOG(INFO) << "Optimizing the pairwise translation estimations.";
	timer.Reset();
	OptimizePairwiseTranslations();
	global_estimator_timings.relative_translation_optimization_time =
		timer.ElapsedTimeInSeconds();

	// Step 6. Filter bad relative translations.
	LOG(INFO) << "Filtering any bad relative translations.";
	timer.Reset();
	FilterRelativeTranslation();
	global_estimator_timings.relative_translation_filtering_time =
		timer.ElapsedTimeInSeconds();

	// Step 7. Estimate global positions.
	LOG(INFO) << "Estimating the positions of all cameras.";
	timer.Reset();
	if (!EstimatePosition()) {
		LOG(WARNING) << "Position estimation failed!";
		summary.success = false;
		return summary;
	}
	LOG(INFO) << m_positions.size()
		<< " camera positions were estimated successfully.";
	global_estimator_timings.position_estimation_time =
		timer.ElapsedTimeInSeconds();

	summary.pose_estimation_time =
		global_estimator_timings.rotation_estimation_time +
		global_estimator_timings.rotation_filtering_time +
		global_estimator_timings.relative_translation_optimization_time +
		global_estimator_timings.relative_translation_filtering_time +
		global_estimator_timings.position_estimation_time;


	// Always triangulate once, then retriangulate and remove outliers depending
	// on the reconstruciton estimator options.
	for (int i = 0; i < m_options.num_retriangulation_iterations + 1; i++) 
	{
		// Step 8. Triangulate features.
		LOG(INFO) << "Triangulating all features, iteration is " << i;
		timer.Reset();
		EstimateStructure();
		summary.triangulation_time += timer.ElapsedTimeInSeconds();


		// Step 9. Bundle Adjustment.
		LOG(INFO) << "Performing bundle adjustment.";
		timer.Reset();
		if (!BundleAdjustment()) {
			summary.success = false;
			LOG(WARNING) << "Bundle adjustment failed!";
			return summary;
		}
		summary.bundle_adjustment_time += timer.ElapsedTimeInSeconds();

		int num_points_removed = RemoveOutlierFeatures(
			m_options.max_reprojection_error_in_pixels,
			m_options.min_triangulation_angle_degrees,
			m_reconstruction);
		LOG(INFO) << num_points_removed << " outlier points were removed.";
	}

	// Set the output parameters.
	GetEstimatedViewsFromReconstruction(*m_reconstruction,
		&summary.estimated_views);
	GetEstimatedTracksFromReconstruction(*m_reconstruction,
		&summary.estimated_tracks);
	summary.success = true;
	summary.total_time = total_timer.ElapsedTimeInSeconds();

	// Output some timing statistics.
	std::ostringstream string_stream;
	string_stream
		<< "Global Reconstruction Estimator timings:"
		<< "\n\tInitial view graph filtering time = "
		<< global_estimator_timings.initial_view_graph_filtering_time
		<< "\n\tCamera intrinsic calibration time = "
		<< summary.camera_intrinsics_calibration_time
		<< "\n\tRotation estimation time = "
		<< global_estimator_timings.rotation_estimation_time
		<< "\n\tRotation filtering time = "
		<< global_estimator_timings.rotation_filtering_time
		<< "\n\tRelative translation optimization time = "
		<< global_estimator_timings.relative_translation_optimization_time
		<< "\n\tRelative translation filtering time = "
		<< global_estimator_timings.relative_translation_filtering_time
		<< "\n\tPosition estimation time = "
		<< global_estimator_timings.position_estimation_time;
	summary.message = string_stream.str();

	LOG(INFO) << "Endding of CGlobalReconstructionEstimator::Estimate";

	return summary;
}

bool CGlobalReconstructionEstimator::FilterInitialViewGraph() 
{
	return true;
}

void CGlobalReconstructionEstimator::CalibrateCameras() 
{
	LOG(INFO) << "Beginning of CGlobalReconstructionEstimator::CalibrateCameras";

	SetCameraIntrinsicsFromPriors(m_reconstruction);

	LOG(INFO) << "Endding of CGlobalReconstructionEstimator::CalibrateCameras";
}

bool CGlobalReconstructionEstimator::EstimateGlobalRotations() 
{
	return true;
}

void CGlobalReconstructionEstimator::FilterRotations() 
{
	return;
}

void CGlobalReconstructionEstimator::OptimizePairwiseTranslations() 
{
	return;
}

void CGlobalReconstructionEstimator::FilterRelativeTranslation() 
{
	return;
}

bool CGlobalReconstructionEstimator::EstimatePosition() 
{
	LOG(INFO) << "Beginning of CGlobalReconstructionEstimator::EstimatePosition";

	// set camera info from view info
	std::vector<ViewId> ids = m_reconstruction->ViewIds();
	for (const auto& id : ids)
	{
		CView* view = m_reconstruction->MutableView(id);
		view->MutableCamera()->SetPosition(view->CameraPostion());
		view->MutableCamera()->SetOrientationFromRotationMatrix(view->CameraOrientationMatrix());
		view->SetEstimated(true);

		LOG(INFO) << "view->Name() = " << view->Name();
		LOG(INFO) << "camera postion is " << view->CameraPostion()(0) << " " << view->CameraPostion()(1) << " " << view->CameraPostion()(2);
		LOG(INFO) << "camera ratation matrix is "
			<< view->CameraOrientationMatrix()(0, 0) << " "
			<< view->CameraOrientationMatrix()(0, 1) << " "
			<< view->CameraOrientationMatrix()(0, 2) << " "
			<< view->CameraOrientationMatrix()(1, 0) << " "
			<< view->CameraOrientationMatrix()(1, 1) << " "
			<< view->CameraOrientationMatrix()(1, 2) << " "
			<< view->CameraOrientationMatrix()(2, 0) << " "
			<< view->CameraOrientationMatrix()(2, 1) << " "
			<< view->CameraOrientationMatrix()(2, 2) << " ";

		LOG(INFO) << "camera postion is " << view->Camera().GetPosition()(0) << " " << view->Camera().GetPosition()(1) << " " << view->Camera().GetPosition()(2);
		LOG(INFO) << "camera ratation matrix is "
			<< view->Camera().GetOrientationAsRotationMatrix()(0, 0) << " "
			<< view->Camera().GetOrientationAsRotationMatrix()(0, 1) << " "
			<< view->Camera().GetOrientationAsRotationMatrix()(0, 2) << " "
			<< view->Camera().GetOrientationAsRotationMatrix()(1, 0) << " "
			<< view->Camera().GetOrientationAsRotationMatrix()(1, 1) << " "
			<< view->Camera().GetOrientationAsRotationMatrix()(1, 2) << " "
			<< view->Camera().GetOrientationAsRotationMatrix()(2, 0) << " "
			<< view->Camera().GetOrientationAsRotationMatrix()(2, 1) << " "
			<< view->Camera().GetOrientationAsRotationMatrix()(2, 2) << " ";

	}
	LOG(INFO);

	// copy camera info to m_orientations and m_positions
	for (const auto& id : ids)
	{
		const CView* view = m_reconstruction->View(id);
		Eigen::Vector3d orientation = view->Camera().GetOrientationAsAngleAxis();
		m_orientations[id] = orientation;
			
		Eigen::Vector3d pos = view->Camera().GetPosition();
		m_positions[id] = pos;
	}
	
	LOG(INFO) << "Beginning of CGlobalReconstructionEstimator::EstimatePosition";



	return true;
}

void CGlobalReconstructionEstimator::EstimateStructure() 
{
	LOG(INFO);

	// Estimate all tracks.
	CTrackEstimator::Options triangulation_options;
	triangulation_options.max_acceptable_reprojection_error_pixels =
		m_options.triangulation_max_reprojection_error_in_pixels;
	triangulation_options.min_triangulation_angle_degrees =
		m_options.min_triangulation_angle_degrees;
	triangulation_options.bundle_adjustment = m_options.bundle_adjust_tracks;
//	triangulation_options.ba_options = SetBundleAdjustmentOptions(options_, 0);
//	triangulation_options.ba_options.verbose = false;
//	triangulation_options.num_threads = options_.num_threads;

	CTrackEstimator track_estimator(triangulation_options, m_reconstruction);
	const CTrackEstimator::Summary summary = track_estimator.EstimateAllTracks();
	
	LOG(INFO);

	return;
}

bool CGlobalReconstructionEstimator::BundleAdjustment() 
{
	// Bundle adjustment.
	//bundle_adjustment_options_ =
	//	SetBundleAdjustmentOptions(options_, positions_.size());
	//const auto& bundle_adjustment_summary =
	//	BundleAdjustReconstruction(bundle_adjustment_options_, reconstruction_);
	//return bundle_adjustment_summary.success;
	return true;
}
