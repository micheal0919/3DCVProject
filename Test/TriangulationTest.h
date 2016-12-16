#pragma once
class CTriangulationTest
{
public:
	CTriangulationTest();
	~CTriangulationTest();

	bool Test();

private:
	bool TriangulateBasicTest();
	bool TriangulationNoiseTest();

	bool TriangulationDLTBasicTest();
	bool TriangulationDLTNoiseTest();
	
	bool TriangulationMidpointNoiseTest();
	bool TriangulationMidpointBasicTest();
	
	bool TriangulationNViewBasicTest();
	bool TriangulationNViewNoiseTest();
	
	bool IsTriangulatedPointInFrontOfCamerasInFront();
	bool IsTriangulatedPointInFrontOfCamerasBehind();
	bool IsTriangulatedPointInFrontOfCamerasOneInFrontOneBehind();

	bool SufficientTriangulationAngleAllSufficient();
	bool SufficientTriangulationAngleAllInsufficient();
	bool SufficientTriangulationAngleSomeInsufficient();
	bool SufficientTriangulationAngleTwoInsufficient();
};

