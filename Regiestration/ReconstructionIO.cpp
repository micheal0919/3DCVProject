#include "ReconstructionIO.h"

#include <opencv2/opencv.hpp>
#include <glog/logging.h>

#include "Track.h"
#include "View.h"

namespace
{
	void CreateEstimatedSubreconstruction(const CReconstruction& input_reconstruction, CReconstruction* estimated_reconstruction)
	{
		*estimated_reconstruction = input_reconstruction;
		const auto view_ids = estimated_reconstruction->ViewIds();
		for (const ViewId view_id : view_ids) {
			const CView* view = estimated_reconstruction->View(view_id);
			if (view == nullptr) {
				continue;
			}

			if (!view->IsEstimated()) {
				estimated_reconstruction->RemoveView(view_id);
			}
		}

		const auto track_ids = estimated_reconstruction->TrackIds();
		for (const TrackId track_id : track_ids) {
			const CTrack* track = estimated_reconstruction->Track(track_id);
			if (track == nullptr) {
				continue;
			}

			if (!track->IsEstimated() || track->NumViews() < 2) {
				estimated_reconstruction->RemoveTrack(track_id);
			}
		}
	}
}

bool WriteReconstruction(const CReconstruction& reconstruction,
	const std::string& output_file)
{
	LOG(INFO) << "Beginning of WriteReconstruction";

	CReconstruction estimated_reconstruction;
	CreateEstimatedSubreconstruction(reconstruction, &estimated_reconstruction);

	std::vector<cv::Point3d> points_3d;
	std::vector<TrackId> ids = estimated_reconstruction.TrackIds();
	for (const auto& id : ids)
	{
		const CTrack* track = estimated_reconstruction.Track(id);
		CHECK_NOTNULL(track);

		Eigen::Vector4d p_h = track->Point();
		Eigen::Vector3d p = p_h.hnormalized();
		cv::Point3d p_cv;
		p_cv.x = p(0);
		p_cv.y = p(1);
		p_cv.z = p(2);
		points_3d.emplace_back(p_cv);
	}

	cv::Mat points3dmatrix = cv::Mat(points_3d);
	cv::FileStorage fs(output_file, cv::FileStorage::WRITE);
	CHECK(fs.isOpened()) << "Fail to open reconstruction file";

	fs << "points_3d" << points3dmatrix;

	fs.release();

	LOG(INFO) << "Endding of WriteReconstruction";

	return true;
}
