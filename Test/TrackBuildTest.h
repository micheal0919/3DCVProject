#pragma once
class CTrackBuildTest
{
public:
	CTrackBuildTest();
	~CTrackBuildTest();

	bool Test();

private:
	bool ConsistentTracksTest();
	bool SingletonTracksTest();
	bool InconsistentTracksTest();
	bool MaxTrackLengthTest();

};

