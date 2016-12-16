#include "TrackBuildTest.h"

#include <glog/logging.h>

#include "Track.h"
#include "View.h"
#include "Reconstruction.h"
#include "TrackBuilder.h"

namespace
{
	// Ensure that each track has been added to every view.
	void VerifyTracks(const CReconstruction& reconstruction) {
		const std::vector<TrackId> track_ids = reconstruction.TrackIds();
		for (const TrackId track_id : track_ids) {
			const CTrack* track = CHECK_NOTNULL(reconstruction.Track(track_id));
			for (const ViewId view_id : track->ViewIds()) {
				const CView* view = CHECK_NOTNULL(reconstruction.View(view_id));
				CHECK_NOTNULL(view->GetFeature(track_id));
			}
		}
	}
}

CTrackBuildTest::CTrackBuildTest()
{
}


CTrackBuildTest::~CTrackBuildTest()
{
}

bool CTrackBuildTest::Test()
{
	LOG(INFO);
	
	CHECK(ConsistentTracksTest());
	CHECK(SingletonTracksTest());
	CHECK(InconsistentTracksTest());
	CHECK(MaxTrackLengthTest());

	LOG(INFO);

	return true;
}

bool CTrackBuildTest::ConsistentTracksTest()
{
	const int kMaxTrackLength = 10;
	const int kNumCorrespondences = 4;

	const ViewId view_ids[kNumCorrespondences][2] = {
		{ 0, 1 }, { 0, 1 }, { 1, 2 }, { 1, 2 }
	};

	CTrackBuilder track_builder(kMaxTrackLength);
	for (int i = 0; i < kNumCorrespondences; i++) {
		track_builder.AddFeatureCorrespondence(view_ids[i][0],
			Feature(i, i),
			view_ids[i][1],
			Feature(i, i));
	}

	CReconstruction reconstruction;
	reconstruction.AddView("0");
	reconstruction.AddView("1");
	reconstruction.AddView("2");
	track_builder.BuildTracks(&reconstruction);
	VerifyTracks(reconstruction);
	CHECK_EQ(reconstruction.NumTracks(), kNumCorrespondences);

	return true;
}

// Singleton tracks.
bool CTrackBuildTest::SingletonTracksTest()
{
	// Having a small max track length will force a singleton track.
	const int kMaxTrackLength = 2;
	const int kNumCorrespondences = 2;

	const ViewId view_ids[kNumCorrespondences][2] = {
		{ 0, 1 }, { 1, 2 } };

	CTrackBuilder track_builder(kMaxTrackLength);
	for (int i = 0; i < kNumCorrespondences; i++) {
		track_builder.AddFeatureCorrespondence(
			view_ids[i][0], Feature(0, 0),
			view_ids[i][1], Feature(0, 0));
	}

	CReconstruction reconstruction;
	reconstruction.AddView("0");
	reconstruction.AddView("1");
	reconstruction.AddView("2");

	track_builder.BuildTracks(&reconstruction);
	VerifyTracks(reconstruction);
	CHECK_EQ(reconstruction.NumTracks(), 1);

	return true;
}

// Inconsistent tracks.
bool CTrackBuildTest::InconsistentTracksTest()
{
	const int kMaxTrackLength = 10;
	const int kNumCorrespondences = 4;

	const ViewId view_ids[kNumCorrespondences][2] = {
		{ 0, 1 }, { 0, 1 }, { 1, 2 }, { 1, 2 }
	};

	CTrackBuilder track_builder(kMaxTrackLength);
	for (int i = 0; i < kNumCorrespondences; i++) {
		track_builder.AddFeatureCorrespondence(
			view_ids[i][0], Feature(0, 0),
			view_ids[i][1], Feature(i + 1, i + 1));
	}

	CReconstruction reconstruction;
	reconstruction.AddView("0");
	reconstruction.AddView("1");
	reconstruction.AddView("2");

	track_builder.BuildTracks(&reconstruction);
	VerifyTracks(reconstruction);
	CHECK_EQ(reconstruction.NumTracks(), 2);

	return true;
}

// Tracks limited by size.
bool CTrackBuildTest::MaxTrackLengthTest()
{
	const int kMaxTrackLength = 2;
	const int kNumViews = 6;

	const ViewId view_ids[kNumViews] = { 0, 1, 2, 3, 4, 5 };

	CTrackBuilder track_builder(kMaxTrackLength);
	for (int i = 0; i < kNumViews - 1; i++) {
		track_builder.AddFeatureCorrespondence(
			view_ids[i], Feature(0, 0),
			view_ids[i + 1], Feature(0, 0));
	}

	CReconstruction reconstruction;
	reconstruction.AddView("0");
	reconstruction.AddView("1");
	reconstruction.AddView("2");
	reconstruction.AddView("3");
	reconstruction.AddView("4");
	reconstruction.AddView("5");

	track_builder.BuildTracks(&reconstruction);
	VerifyTracks(reconstruction);
	CHECK_EQ(reconstruction.NumTracks(), 3);

	return true;
}
