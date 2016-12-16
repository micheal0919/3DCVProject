#include <gflags/gflags.h>
#include <glog/logging.h>

#include "Detector.h"

DEFINE_string(option, "test", "the option of the project neeed to do, pose, reconstruction, detection");
DEFINE_string(camera_intrin_file_path, "../data/calibration/camera_params.txt", "camera_intrin_file_path");
DEFINE_string(marker_world_file_path, "../data/markers/worldflat-1.xml", "marker_world_file_path");
DEFINE_string(image_path, "../data/images/", "image_path");
DEFINE_string(image_info_file_path, "../data/images/image_infos.xml", "image_info_file_path");
DEFINE_int32(num_image, 5, "num_image");

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
	CDetector detector(options);
	
	if (!detector.Detect())
	{
		LOG(ERROR) << "error in detection";
		return -1;
	}
	
	LOG(INFO) << "Detect camera poses sucessfully";

	LOG(INFO) << "end of main function";

	return 0;
}
