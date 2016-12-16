#ifndef __TRACK_H__
#define __TRACK_H__

#include <unordered_set>
#include <Eigen/Core>
#include "Types.h"

// A track contains information about a 3D point and the views that observe the point.
class CTrack {
public:
	EIGEN_MAKE_ALIGNED_OPERATOR_NEW

	CTrack()
		: m_is_estimated(false) {
		m_point.setZero();
		m_color.setZero();
	}
	~CTrack() {}

	int NumViews() const
	{
		return m_view_ids.size();
	}

	void SetEstimated(const bool is_estimated)
	{
		m_is_estimated = is_estimated;
	}

	bool IsEstimated() const
	{
		return m_is_estimated;
	}

	const Eigen::Vector4d& Point() const
	{
		return m_point;
	}

	Eigen::Vector4d* MutablePoint()
	{
		return &m_point;
	}

	const Eigen::Matrix<uint8_t, 3, 1>& Color() const
	{
		return m_color;
	}

	Eigen::Matrix<uint8_t, 3, 1>* MutableColor()
	{
		return &m_color;
	}

	void AddView(const ViewId view_id)
	{
		m_view_ids.insert(view_id);
	}

	bool RemoveView(const ViewId view_id)
	{
		return m_view_ids.erase(view_id);
	}

	const std::unordered_set<ViewId>& ViewIds() const
	{
		return m_view_ids;
	}

private:
	bool m_is_estimated;
	std::unordered_set<ViewId> m_view_ids;
	Eigen::Vector4d m_point;
	Eigen::Matrix<uint8_t, 3, 1> m_color;
};

#endif // __TRACK_H__
