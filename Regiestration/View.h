#ifndef __VIEW_H__
#define __VIEW_H__

#include <Eigen/Core>

#include <string>
#include <unordered_map>
#include <vector>

#include "Types.h"
#include "MapUtils.h"
#include "Camera.h"


class CView {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	CView()
		: m_name("")
		, m_is_estimated(false)
	{
	}

	explicit CView(const std::string& name)
		: m_name(name)
		, m_is_estimated(false)
	{
	}

	~CView() {}

	const std::string& Name() const
	{
		return m_name;
	}

	void SetEstimated(bool is_estimated)
	{
		m_is_estimated = is_estimated;
	}

	bool IsEstimated() const
	{
		return m_is_estimated;
	}

	const CCamera& Camera() const 
	{
		return m_camera;
	}

	CCamera* MutableCamera() 
	{
		return &m_camera;
	}

	const struct CameraIntrinsicsPrior& CameraIntrinsicsPrior() const
	{
		return m_camera_intrinsics_prior;
	}

	struct CameraIntrinsicsPrior* MutableCameraIntrinsicsPrior() 
	{
		return &m_camera_intrinsics_prior;
	}

	Eigen::Matrix3d CameraOrientationMatrix() const
	{
		return m_camera_orientation;
	}

	Eigen::Matrix3d* MutableCameraOrientationMatrix()
	{
		return &m_camera_orientation;
	}

	Eigen::Vector3d CameraPostion() const
	{
		return m_camera_postion;
	}

	Eigen::Vector3d* MutableCameraPostion()
	{
		return &m_camera_postion;
	}

	int NumFeatures() const
	{
		return m_features.size();
	}

	std::vector<TrackId> TrackIds() const
	{
		std::vector<TrackId> track_ids;
		track_ids.reserve(m_features.size());
		for (const auto& track : m_features) 
		{
			track_ids.emplace_back(track.first);
		}
		return track_ids;
	}

	const Feature* GetFeature(const TrackId track_id) const
	{
		return FindOrNull(m_features, track_id);
	}

	void AddFeature(const TrackId track_id, const Feature& feature)
	{
		m_features[track_id] = feature;
	}

	bool RemoveFeature(const TrackId track_id)
	{
		return m_features.erase(track_id) > 0;
	}

private:
	std::string m_name;
	bool m_is_estimated;
	CCamera m_camera;
	struct CameraIntrinsicsPrior m_camera_intrinsics_prior;
	Eigen::Matrix3d m_camera_orientation;
	Eigen::Vector3d m_camera_postion;
	std::unordered_map<TrackId, Feature> m_features;
};

#endif // __VIEW_H__
