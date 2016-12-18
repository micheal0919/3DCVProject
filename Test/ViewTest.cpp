#include "ViewTest.h"

#include <string>

#include <glog/logging.h>

#include "View.h"

CViewTest::CViewTest()
{
}


CViewTest::~CViewTest()
{
}

bool CViewTest::Test()
{
	LOG(INFO) << "Beginning of CViewTest::Test";

	CHECK(Name());
	CHECK(Estimated());
	CHECK(Features());
	CHECK(TrackIds());

	LOG(INFO) << "Endding of CViewTest::Test";

	return true;
}

bool CViewTest::Name()
{
	const std::string kName = "0";
	CView view(kName);
	CHECK_EQ(view.Name(), kName);

	return true;
}

bool CViewTest::Estimated()
{
	CView view;
	CHECK(!view.IsEstimated());
	view.SetEstimated(true);
	CHECK(view.IsEstimated());
	view.SetEstimated(false);
	CHECK(!view.IsEstimated());

	return true;
}

bool CViewTest::Features()
{
	CView view;
	const std::vector<TrackId> track_ids = { 0, 1, 2 };
	const std::vector<Feature> features = { Feature(0, 0),
		Feature(1, 1),
		Feature(2, 2) };
	// Test that no features exist.
	CHECK_EQ(view.NumFeatures(), 0);

	// Add features.
	for (int i = 0; i < track_ids.size(); i++) {
		view.AddFeature(track_ids[i], features[i]);
		const Feature* feature = view.GetFeature(track_ids[i]);
		CHECK(feature);
		CHECK_EQ(*feature, features[i]);
		CHECK_EQ(view.NumFeatures(), i + 1);
	}

	for (int i = 0; i < track_ids.size(); i++) {
		view.RemoveFeature(track_ids[i]);
		const Feature* feature = view.GetFeature(track_ids[i]);
		CHECK(!feature);
		CHECK_EQ(view.NumFeatures(), 3 - i - 1);
	}

	return true;
}


bool CViewTest::TrackIds()
{
	CView view;
	const std::vector<TrackId> track_ids = { 0, 1, 2 };
	const std::vector<Feature> features = { Feature(0, 0),
		Feature(1, 1),
		Feature(2, 2) };
	// Add features.
	for (int i = 0; i < track_ids.size(); i++) {
		view.AddFeature(track_ids[i], features[i]);
	}

	// Make sure the track ids are equivalent.
	std::vector<TrackId> temp_track_ids = view.TrackIds();
	std::sort(temp_track_ids.begin(), temp_track_ids.end());
	for (int i = 0; i < track_ids.size(); i++) {
		CHECK_EQ(track_ids[i], temp_track_ids[i]);
	}

	return true;
}
