#include <gflags/gflags.h>
#include <glog/logging.h>

#include "test.h"

int main(int argc, char *argv[])
{
	gflags::ParseCommandLineFlags(&argc, &argv, true);

	// set google glog
	{
		google::InitGoogleLogging("LogFileName");
		// 0: log to file; 1: log to std io
		FLAGS_logtostderr = 1;
		FLAGS_logbuflevel = 1;
		google::SetLogDestination(0, "./LOG.");
	}

	LOG(INFO) << "start to test";

	Test test;
	test.TestAll();

	LOG(INFO) << "end to test";

	return 0;
}