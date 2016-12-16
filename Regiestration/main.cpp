#include <iostream>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/opencv.hpp>

#include "ReconstructionBuilder.h"

using namespace std;

DEFINE_string(option, "test", "the option of the project neeed to do, pose, reconstruction, detection");
//DEFINE_string(camera_intrin_file_path, "../data/calibration/camera_params.txt", "camera_intrin_file_path");
//DEFINE_string(marker_world_file_path, "../data/markers/worldflat-1.xml", "marker_world_file_path");
DEFINE_string(image_path, "../data/images/", "image_path");
DEFINE_string(image_info_file_path, "../data/images/image_infos.xml", "image_info_file_path");
DEFINE_int32(num_image, 5, "num_image");
DEFINE_int32(num_keypoint, 200, "num_keypoint of each image");

DEFINE_string(output_reconstruction, "../", "output_reconstruction");

int main(int argc, char* argv[])
{
	gflags::ParseCommandLineFlags(&argc, &argv, true);

	// set google glog
	{
		google::InitGoogleLogging("LogFileName");
		// 0: log to file; 1: log to std io
		FLAGS_logtostderr = 0;
		FLAGS_logbuflevel = 1;
		google::SetLogDestination(0, "./Registration.");
	}

	LOG(INFO) << "Reconstruct the 3D model" << endl;
	ReconstructionBuilderOptions options;
	options.image_path = FLAGS_image_path;
	options.num_image = FLAGS_num_image;
	options.image_info_file_path = FLAGS_image_info_file_path;
	options.feam_options.num_keypoint = FLAGS_num_keypoint;
	LOG(INFO) << std::endl;

	CReconstructionBuilder builder(options);
	LOG(INFO) << std::endl;

	builder.AddImages();
	LOG(INFO) << std::endl;

	builder.ExtractAndMatchFeatures();
	LOG(INFO) << std::endl;

	std::vector<CReconstruction*> reconstructions;
	builder.BuildReconstruction(&reconstructions);
	LOG(INFO) << std::endl;

	for (int i = 0; i < reconstructions.size(); i++) {
		const std::string output_file = FLAGS_output_reconstruction;
		LOG(INFO) << "Writing reconstruction " << i << " to " << output_file;
		CHECK(WriteReconstruction(*reconstructions[i], output_file))
			<< "Could not write reconstruction to file.";
	}

	LOG(INFO) << "Reconstruct the 3D model sucessfully" << endl;

	LOG(INFO) << "end of main function" << endl;

	return 0;
}
