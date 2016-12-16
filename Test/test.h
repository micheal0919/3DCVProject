#pragma once

class Test
{
public:
	Test();
	~Test();
	void TestAll();

private:
	bool surf_test();
	bool tiny_xml_test();
	bool robust_matcher_test();
	bool track_build_test();
	bool camera_test();
};

