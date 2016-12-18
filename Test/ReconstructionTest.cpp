#include "ReconstructionTest.h"

#include <vector>

#include <glog/logging.h>

#include "Reconstruction.h"

namespace
{
	const std::vector<std::string> view_names = { "1", "2", "3" };
	const std::vector<Feature> features = { Feature(1, 1),
		Feature(2, 2),
		Feature(3, 3) };
}

CReconstructionTest::CReconstructionTest()
{
}


CReconstructionTest::~CReconstructionTest()
{
}

bool CReconstructionTest::Test()
{
	LOG(INFO) << "Beginning of CReconstructionTest::Test";

	CHECK(ViewIdFromNameValid());
	CHECK(ViewIdFromNameInvalid());
	CHECK(AddView());
	CHECK(RemoveView());
	CHECK(GetViewValid());
	CHECK(GetViewValidInvalid());
	CHECK(AddTrackValid());
	CHECK(AddTrackInvalid());
	CHECK(RemoveTrackValid());
	CHECK(RemoveTrackInvalid());
	CHECK(GetTrackValid());
	CHECK(GetTrackInvalid());

	LOG(INFO) << "Endding of CReconstructionTest::Test";

	return true;
}

bool CReconstructionTest::ViewIdFromNameValid()
{
	CReconstruction reconstruction;
	const ViewId gt_view_id = reconstruction.AddView(view_names[0]);

	const ViewId view_id = reconstruction.ViewIdFromName(view_names[0]);
	CHECK_EQ(gt_view_id, view_id);

	return true;
}

bool CReconstructionTest::ViewIdFromNameInvalid()
{
	CReconstruction reconstruction;
	CHECK_EQ(reconstruction.ViewIdFromName(view_names[0]), kInvalidViewId);

	return true;
}

bool CReconstructionTest::AddView()
{
	CReconstruction reconstruction;
	const ViewId view_id = reconstruction.AddView(view_names[0]);
	CHECK_NE(view_id, kInvalidViewId);
	CHECK_EQ(reconstruction.NumViews(), 1);
	CHECK_EQ(reconstruction.NumTracks(), 0);
	CHECK_EQ(reconstruction.AddView(view_names[0]), kInvalidViewId);

	return true;
}

bool CReconstructionTest::RemoveView()
{
	CReconstruction reconstruction;
	const ViewId view_id1 = reconstruction.AddView(view_names[0]);
	const ViewId view_id2 = reconstruction.AddView(view_names[1]);
	CHECK_EQ(reconstruction.NumViews(), 2);

	CHECK(reconstruction.RemoveView(view_id1));
	CHECK_EQ(reconstruction.NumViews(), 1);
	CHECK_EQ(reconstruction.ViewIdFromName(view_names[0]), kInvalidViewId);
	CHECK(!reconstruction.View(view_id1));

	CHECK(reconstruction.RemoveView(view_id2));
	CHECK_EQ(reconstruction.NumViews(), 0);
	CHECK_EQ(reconstruction.ViewIdFromName(view_names[1]), kInvalidViewId);
	CHECK(!reconstruction.View(view_id2));

	CHECK(!reconstruction.RemoveView(kInvalidViewId));
	CHECK(!reconstruction.RemoveView(view_id1));

	return true;
}

bool CReconstructionTest::GetViewValid()
{
	CReconstruction reconstruction;
	const ViewId view_id = reconstruction.AddView(view_names[0]);
	CHECK_NE(view_id, kInvalidViewId);

	const CView* const_view = reconstruction.View(view_id);
	CHECK_NOTNULL(const_view);

	CView* mutable_view = reconstruction.MutableView(view_id);
	CHECK_NOTNULL(mutable_view);

	return true;
}

bool CReconstructionTest::GetViewValidInvalid()
{
	CReconstruction reconstruction;
	static const ViewId view_id = 0;
	const CView* const_view = reconstruction.View(view_id);
	CHECK(!const_view);

	CView* mutable_view = reconstruction.MutableView(view_id);
	CHECK(!mutable_view);

	return true;
}

bool CReconstructionTest::AddTrackValid()
{
	CReconstruction reconstruction;

	const std::vector<std::pair<ViewId, Feature> > track = {
		{ 0, features[0] }, { 1, features[1] }
	};
	CHECK_NE(reconstruction.AddView(view_names[0]), kInvalidViewId);
	CHECK_NE(reconstruction.AddView(view_names[1]), kInvalidViewId);

	const TrackId track_id = reconstruction.AddTrack(track);
	CHECK_NE(track_id, kInvalidTrackId);
	CHECK_NOTNULL(reconstruction.Track(track_id));
	CHECK_EQ(reconstruction.NumTracks(), 1);

	return true;
}

bool CReconstructionTest::AddTrackInvalid()
{
	CReconstruction reconstruction;

	// Should fail with less than two views.
	const std::vector<std::pair<ViewId, Feature> > small_track = {
		{ 0, features[0] }
	};
	CHECK_NE(reconstruction.AddView(view_names[0]), kInvalidViewId);
	CHECK_EQ(reconstruction.AddTrack(small_track), kInvalidTrackId);
	CHECK_EQ(reconstruction.NumTracks(), 0);

	return true;
}

bool CReconstructionTest::RemoveTrackValid()
{
	CReconstruction reconstruction;

	const std::vector<std::pair<ViewId, Feature> > track = {
		{ 0, features[0] }, { 1, features[1] }
	};

	// Should be able to successfully remove the track.
	CHECK_NE(reconstruction.AddView(view_names[0]), kInvalidViewId);
	CHECK_NE(reconstruction.AddView(view_names[1]), kInvalidViewId);
	const TrackId track_id = reconstruction.AddTrack(track);
	CHECK(reconstruction.RemoveTrack(track_id));

	return true;
}

bool CReconstructionTest::RemoveTrackInvalid()
{
	CReconstruction reconstruction;

	// Should return false when trying to remove a track not in the
	// reconstruction.
	CHECK(!reconstruction.RemoveTrack(kInvalidTrackId));

	return true;
}

bool CReconstructionTest::GetTrackValid()
{
	CReconstruction reconstruction;
	const std::vector<std::pair<ViewId, Feature> > track = {
		{ 0, features[0] }, { 1, features[1] }
	};
	CHECK_NE(reconstruction.AddView(view_names[0]), kInvalidViewId);
	CHECK_NE(reconstruction.AddView(view_names[1]), kInvalidViewId);
	const TrackId track_id = reconstruction.AddTrack(track);
	CHECK_NE(track_id, kInvalidTrackId);

	const CTrack* const_track = reconstruction.Track(track_id);
	CHECK_NOTNULL(const_track);

	CTrack* mutable_track = reconstruction.MutableTrack(track_id);
	CHECK_NOTNULL(mutable_track);

	return true;
}

bool CReconstructionTest::GetTrackInvalid()
{
	CReconstruction reconstruction;
	const std::vector<std::pair<ViewId, Feature> > track = {};
	const TrackId track_id = reconstruction.AddTrack(track);
	CHECK_EQ(track_id, kInvalidTrackId);

	const CTrack* const_track = reconstruction.Track(track_id);
	CHECK(!const_track);

	CTrack* mutable_track = reconstruction.MutableTrack(track_id);
	CHECK(!mutable_track);

	return true;
}
