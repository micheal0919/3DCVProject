#pragma once
class CCameraTest
{
public:
	CCameraTest();
	~CCameraTest();

	bool Test();

private:
	bool ProjectionMatrixTest();
	bool InternalParameterGettersAndSetters();
	bool ExternalParameterGettersAndSetters();
	bool ReprojectionNoDistortion();
};

