#include "TrackTest.h"

#include <glog/logging.h>
#include <Eigen/Core>

#include "Track.h"
#include "MapUtils.h"

CTrackTest::CTrackTest()
{
}


CTrackTest::~CTrackTest()
{
}

bool CTrackTest::Test()
{
	LOG(INFO) << "Beginning of CTrackTest::Test";

	CHECK(Default());
	CHECK(Estimated());
	CHECK(Views());
	CHECK(ViewIds());

	LOG(INFO) << "Beginning of CTrackTest::Test";

	return true;
}

bool CTrackTest::Default()
{
	CTrack track;
	CHECK_EQ(track.NumViews(), 0);
	CHECK_EQ(track.Point(), Eigen::Vector4d::Zero());

	return true;
}

bool CTrackTest::Estimated()
{
	CTrack track;
	CHECK(!track.IsEstimated());
	track.SetEstimated(true);
	CHECK(track.IsEstimated());
	track.SetEstimated(false);
	CHECK(!track.IsEstimated());

	return true;
}

bool CTrackTest::Views()
{
	CTrack track;
	const std::vector<ViewId> view_ids = { 0, 1, 2 };
	// Test that no views exist.
	CHECK_EQ(track.NumViews(), 0);

	// Add views.
	for (int i = 0; i < view_ids.size(); i++) {
		track.AddView(view_ids[i]);
		CHECK_EQ(track.NumViews(), i + 1);
	}

	for (int i = 0; i < view_ids.size(); i++) {
		track.RemoveView(view_ids[i]);
		CHECK_EQ(track.NumViews(), 3 - i - 1);
	}

	return true;
}

bool CTrackTest::ViewIds()
{
	CTrack track;
	const std::vector<ViewId> view_ids = { 0, 1, 2 };

	// Add views.
	for (int i = 0; i < view_ids.size(); i++) {
		track.AddView(view_ids[i]);
	}

	// Make sure the track ids are equivalent.
	std::unordered_set<ViewId> temp_view_ids = track.ViewIds();
	CHECK_EQ(temp_view_ids.size(), 3);
	for (int i = 0; i < view_ids.size(); i++) {
		CHECK(ContainsKey(temp_view_ids, view_ids[i]));
	}

	return true;
}
