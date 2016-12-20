#include <gflags/gflags.h>
#include <glog/logging.h>

#include "PoseEstimator.h"

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
		FLAGS_logtostderr = 0;
		FLAGS_logbuflevel = 1;
		google::SetLogDestination(0, "./LOG.");
	}

	LOG(INFO) << "Estimate camera poses";
	CPoseEstimator::Options options;
	options.camera_intrin_file_path = FLAGS_camera_intrin_file_path;
	options.marker_world_file_path = FLAGS_marker_world_file_path;
	options.image_path = FLAGS_image_path;
	options.image_info_file_path = FLAGS_image_info_file_path;
	options.num_image = FLAGS_num_image;
	CPoseEstimator estimator(options);
	if (!estimator.EstimatePoses())
	{
		LOG(ERROR) << "error in take images from web camera";
		return -1;
	}
	LOG(INFO) << "Estimate camera poses sucessfully";


	LOG(INFO) << "end of main function";

	return 0;
}