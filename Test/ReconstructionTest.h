#pragma once
class CReconstructionTest
{
public:
	CReconstructionTest();
	~CReconstructionTest();

	bool Test();

private:
	bool ViewIdFromNameValid();
	bool ViewIdFromNameInvalid();
	bool AddView();
	bool RemoveView();
	bool GetViewValid();
	bool GetViewValidInvalid();
	bool AddTrackValid();
	bool AddTrackInvalid();
	bool RemoveTrackValid();
	bool RemoveTrackInvalid();
	bool GetTrackValid();
	bool GetTrackInvalid();

};

