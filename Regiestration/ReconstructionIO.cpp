#include "ReconstructionIO.h"

#include <string>
#include <vector>

#include <opencv2/opencv.hpp>
#include <glog/logging.h>

#include "Types.h"
#include "Track.h"
#include "View.h"
#include "RobustMatcher.h"

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

	void ComputeDescriptorOfEstimatedTracks(const CReconstruction& reconstruction, cv::Mat& descriptors)
	{
		LOG(INFO) << "Begninning of ComputeDescriptorOfEstimatedTracks";
		
		std::vector<TrackId> track_ids = reconstruction.TrackIds();
		std::vector<bool> track_flags;

		for (size_t i = 0; i < track_ids.size(); i++)
		{
			track_flags.push_back(false);
		}

		CRobustMatcher matcher;

		std::vector<ViewId> view_ids = reconstruction.ViewIds();
		for (const ViewId view_id : view_ids)
		{
			const CView *view = reconstruction.View(view_id);
			cv::Mat img = cv::imread(view->Name());
			
			cv::namedWindow("imageshow", cv::WINDOW_AUTOSIZE);
			cv::imshow("imageshow", img);
			cv::waitKey();

			std::vector<cv::KeyPoint> kpts;
			cv::Mat des;
			matcher.ComputeKeyPoints(img, kpts);
			matcher.ComputeDescriptors(img, kpts, des);

			for (size_t i = 0; i < track_ids.size(); i++)
			{
				
				// if the track descriptor has been added to result, then go to next track
				if (true == track_flags[i])
				{
					continue;
				}

				const TrackId track_id = track_ids[i];
				const Feature* feature = view->GetFeature(track_id);
				// if the view do not reference to the track, then go to the next track
				if (NULL == feature)
				{
					continue;
				}

				const CTrack* track = reconstruction.Track(track_id);
				// if the track is not estimated, then set the flags and go to the next track
				if (!track->IsEstimated())
				{
					track_flags[i] = true;
					continue;
				}

				for (size_t k = 0; k < kpts.size(); k++)
				{
					// if the track references to the view, and the descriptor to the result, and the set the flag
					if (feature->x == kpts[k].pt.x && feature->y == kpts[k].pt.y)
					{
						descriptors.push_back(des.row(k));
						track_flags[i] = true;
					}
				}
				
			}
		}

		LOG(INFO) << "The num of descriptors is " << descriptors.rows;

		for (size_t i = 0; i < track_flags.size(); i++)
		{
			if (false == track_flags[i])
			{
				LOG(INFO) << "The error num is " << i;
			}
		}

		LOG(INFO) << "Endding of ComputeDescriptorOfEstimatedTracks";
	}
}

bool WriteReconstruction(const CReconstruction& reconstruction,
	const std::string& output_file)
{
	LOG(INFO) << "Beginning of WriteReconstruction";

	CReconstruction estimated_reconstruction;
	CreateEstimatedSubreconstruction(reconstruction, &estimated_reconstruction);

	cv::Mat des;
	ComputeDescriptorOfEstimatedTracks(estimated_reconstruction, des);
	LOG(INFO);

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
	fs << "descriptors" << des;

	fs.release();

	LOG(INFO) << "Endding of WriteReconstruction";

	return true;
}
