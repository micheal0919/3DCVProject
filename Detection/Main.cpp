#include <gflags/gflags.h>
#include <glog/logging.h>

#include "Detector.h"

DEFINE_string(input_reconstruction, "../data/images/3DReconstruction.yml", "output_reconstruction");

int main(int argc, char* argv[])
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

	LOG(INFO) << "Detect camera poses";

	CDetector::Options options;
	options.yml_read_path = FLAGS_input_reconstruction;
	CDetector detector(options);
	
	if (!detector.Init())
	{
		LOG(ERROR) << "Fail to init detector";
		return -1;
	}

	if (!detector.Detect())
	{
		LOG(ERROR) << "error in detection";
		return -1;
	}
	
	LOG(INFO) << "Detect camera poses sucessfully";

	LOG(INFO) << "end of main function";

	return 0;
}
