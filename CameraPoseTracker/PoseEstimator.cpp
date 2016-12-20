#include "PoseEstimator.h"

#include <stdio.h>
#include <conio.h>

#include <iostream>
#include <fstream>
#include <string>
#include <iomanip>
#include <fstream>

#include <Windows.h>

#include <glog/logging.h>

#include <tinyxml.h>

#include "ImageInfoIO.h"

namespace
{
	std::string GetImageNameFromNum(int num)
	{
		char buf[32];
		sprintf(buf, "%06d.jpg", num);
		std::string name = buf;

		return name;
	}
}

CPoseEstimator::CPoseEstimator(const Options& option)
: m_options(option)
//, m_cap(-1) // open the default camera
{
}

CPoseEstimator::~CPoseEstimator()
{
}

bool CPoseEstimator::Init()
{
	//get image properties
	char config_default[] = "<?xml version=\"1.0\" encoding=\"UTF-8\"?><dsvl_input><camera show_format_dialog=\"false\" friendly_name=\"\" frame_width=\"640\" frame_height=\"480\"><pixel_format><RGB32 flip_h=\"false\" flip_v=\"true\"/></pixel_format></camera></dsvl_input>";

	// check if we succeeded
	if (!m_cap.init(config_default))
	{
		LOG(ERROR) << "Fail to open video capture" << std::endl;
		return false;
	}

	//while (true)
	//{
	//	IplImage * live_video = m_cap.getImage();
	//	LOG(INFO) << "Image size " << live_video->width << "x" << live_video->height << ", " << live_video->nChannels << " channels/pixel" << ", " << live_video->depth << " bits/channel" << std::endl;
	//	cvShowImage("MarkerTrackerTest", live_video);
	//}

	unsigned int w = m_cap.width();
	unsigned int h = m_cap.height();
	unsigned int d = 8; //TODO: read this from video
	unsigned int c = 4; //TODO: read this from video
	LOG(INFO) << "Image size " << w << "x" << h << ", " << c << " channels/pixel" << ", " << d << " bits/channel" << std::endl;

	//call start once at the beginning
	std::string error_message;
	LOG(INFO) << "camera_intrin_file_path = " << m_options.camera_intrin_file_path;
	LOG(INFO) << "marker_world_file_path = " << m_options.marker_world_file_path;
	
	// test the file path
	{
		//FILE* camera_file = fopen(m_options.camera_intrin_file_path.c_str(), "r");
		//CHECK_NOTNULL(camera_file);
		//char* buffer[256];
		//fread(buffer, 1, 256, camera_file);
		//LOG(INFO) << buffer;
		//FILE* marker_file = fopen(m_options.marker_world_file_path.c_str(), "r");
		//CHECK_NOTNULL(marker_file);

		//std::string str;
		//std::ifstream infile;
		//infile.open(m_options.camera_intrin_file_path);
		//while (!infile.eof()) // To get you all the lines.
		//{
		//	std::getline(infile, str); // Saves the line in STRING.
		//	std::cout << str << std::endl; // Prints our STRING.
		//}
		//infile.close();

		//infile.open(m_options.marker_world_file_path);
		//while (!infile.eof()) // To get you all the lines.
		//{
		//	std::getline(infile, str); // Saves the line in STRING.
		//	std::cout << str << std::endl; // Prints our STRING.
		//}
		//infile.close();

		//// open xml file
		//TiXmlDocument* myDocument = new TiXmlDocument(m_options.marker_world_file_path.c_str());
		//CHECK(myDocument->LoadFile()) << "Fail to load image infos xml file from disk";
		//// load root node
		//TiXmlElement* RootElement = myDocument->RootElement();
		//TiXmlHandle rootHandle(RootElement);

	}

	bool res = m_marker_tracker.start(m_options.camera_intrin_file_path, m_options.marker_world_file_path, false, false, w, h, d, c, error_message);
	if (!res)
	{
		LOG(ERROR) << "Fail to init marker tracker, and the error message is " << error_message << std::endl;
		return false;
	}

	LOG(INFO) << "Succeed to init maker tracker" << std::endl;
		
	return true;
}

bool CPoseEstimator::EstimatePoses()
{
	LOG(INFO) << "Beginning of EstimatePoses";
	if (!Init())
	{
		LOG(ERROR) << "Fail to init marker tracker" << std::endl;
		return false;
	}

	
	
	unsigned int num_rigid_bodies = m_marker_tracker.getRigidBodyCount();
	LOG(INFO) << "The num of rigid bodies is " << num_rigid_bodies;

	//get the K matrix
	cv::Mat K_mat(3, 3, CV_64FC1);
	
	double K_matrix[9];
	m_marker_tracker.getKMatrix(K_matrix);

	for (unsigned int i = 0; i<3; i++)
	{
		for (unsigned int j = 0; j<3; j++)
		{
			K_mat.at<double>(i, j) = K_matrix[i + j * 3];
		}
	}
	LOG(INFO) << "K mat is " << K_mat;

	//for (int i = 0; i < 9; i++)
	//{
	//	int x = i / 3;
	//	int y = i % 3;
	//	K_mat.at<double>(x, y) = K_matrix[i];
	//}
	

	CvMat* Kmat = cvCreateMat(3, 3, CV_64FC1);
	CvMat* Rmat = cvCreateMat(3, 3, CV_64FC1);
	CvMat* tvec = cvCreateMat(3, 1, CV_64FC1);
	for (unsigned int i = 0; i<3; i++)
	{
		for (unsigned int j = 0; j<3; j++)
		{
			cvmSet(Kmat, i, j, K_matrix[i + j * 3]);
		}
	}

	const double scale = 30.0;

	CvMat* point0 = cvCreateMat(3, 1, CV_64FC1);
	CvMat* pointX = cvCreateMat(3, 1, CV_64FC1);
	CvMat* pointY = cvCreateMat(3, 1, CV_64FC1);
	CvMat* pointZ = cvCreateMat(3, 1, CV_64FC1);

	cvmSet(point0, 0, 0, 0);
	cvmSet(point0, 1, 0, 0);
	cvmSet(point0, 2, 0, 0);

	cvmSet(pointX, 0, 0, scale);
	cvmSet(pointX, 1, 0, 0);
	cvmSet(pointX, 2, 0, 0);

	cvmSet(pointY, 0, 0, 0);
	cvmSet(pointY, 1, 0, scale);
	cvmSet(pointY, 2, 0, 0);

	cvmSet(pointZ, 0, 0, 0);
	cvmSet(pointZ, 1, 0, 0);
	cvmSet(pointZ, 2, 0, scale);

	CvMat* projected0 = cvCreateMat(3, 1, CV_64FC1);
	CvMat* projectedX = cvCreateMat(3, 1, CV_64FC1);
	CvMat* projectedY = cvCreateMat(3, 1, CV_64FC1);
	CvMat* projectedZ = cvCreateMat(3, 1, CV_64FC1);

	//main video loop
	bool * marker_visible_flags = new bool[num_rigid_bodies];
	double * modelview_matrices = new double[16 * num_rigid_bodies];

	int image_counter = 0;

	LOG(INFO) << "Start the main loop";

	std::vector<ImageInfo> infos;
	//The main loop
	while (image_counter < m_options.num_image)
	{
		//call getData to get the current image and current matrix

		IplImage * live_video = m_cap.getImage();
		LOG(INFO) << "Get the live video form camera";

		//call getData to get the current matrix
		m_marker_tracker.getData(marker_visible_flags, modelview_matrices, reinterpret_cast<unsigned char*>(live_video->imageData));
		LOG(INFO) << "Get the data form marker tracker";

		bool live_show = false;

		for (unsigned int i = 0; i < num_rigid_bodies; i++)
		{
			LOG(INFO) << std::endl;
			if (marker_visible_flags[i])
			{
				LOG(INFO) << std::endl;
				std::string image_name = GetImageNameFromNum(image_counter);
				LOG(INFO) << "the image name is " << image_name;

				//first column of rotation
				cvmSet(Rmat, 0, 0, modelview_matrices[0 + 16 * i]);
				cvmSet(Rmat, 1, 0, modelview_matrices[1 + 16 * i]);
				cvmSet(Rmat, 2, 0, modelview_matrices[2 + 16 * i]);

				//second column of rotation
				cvmSet(Rmat, 0, 1, modelview_matrices[4 + 16 * i]);
				cvmSet(Rmat, 1, 1, modelview_matrices[5 + 16 * i]);
				cvmSet(Rmat, 2, 1, modelview_matrices[6 + 16 * i]);

				//third column of rotation
				cvmSet(Rmat, 0, 2, modelview_matrices[8 + 16 * i]);
				cvmSet(Rmat, 1, 2, modelview_matrices[9 + 16 * i]);
				cvmSet(Rmat, 2, 2, modelview_matrices[10 + 16 * i]);

				//translation
				cvmSet(tvec, 0, 0, modelview_matrices[12 + 16 * i]);
				cvmSet(tvec, 1, 0, modelview_matrices[13 + 16 * i]);
				cvmSet(tvec, 2, 0, modelview_matrices[14 + 16 * i]);

				LOG(INFO) << std::endl;

				cvMatMul(Rmat, point0, projected0);
				cvAdd(projected0, tvec, projected0);
				cvMatMul(Kmat, projected0, projected0);
				cvScale(projected0, projected0, 1.0 / cvmGet(projected0, 2, 0));

				cvMatMul(Rmat, pointX, projectedX);
				cvAdd(projectedX, tvec, projectedX);
				cvMatMul(Kmat, projectedX, projectedX);
				cvScale(projectedX, projectedX, 1.0 / cvmGet(projectedX, 2, 0));

				cvMatMul(Rmat, pointY, projectedY);
				cvAdd(projectedY, tvec, projectedY);
				cvMatMul(Kmat, projectedY, projectedY);
				cvScale(projectedY, projectedY, 1.0 / cvmGet(projectedY, 2, 0));

				cvMatMul(Rmat, pointZ, projectedZ);
				cvAdd(projectedZ, tvec, projectedZ);
				cvMatMul(Kmat, projectedZ, projectedZ);
				cvScale(projectedZ, projectedZ, 1.0 / cvmGet(projectedZ, 2, 0));

				LOG(INFO) << std::endl;


				IplImage* display = cvCloneImage(live_video);

				//draw coordinate system
				cvLine(display, cvPoint(cvmGet(projected0, 0, 0), cvmGet(projected0, 1, 0)), cvPoint(cvmGet(projectedX, 0, 0), cvmGet(projectedX, 1, 0)), CV_RGB(255, 0, 0), 3, 8);
				cvLine(display, cvPoint(cvmGet(projected0, 0, 0), cvmGet(projected0, 1, 0)), cvPoint(cvmGet(projectedY, 0, 0), cvmGet(projectedY, 1, 0)), CV_RGB(0, 255, 0), 3, 8);
				cvLine(display, cvPoint(cvmGet(projected0, 0, 0), cvmGet(projected0, 1, 0)), cvPoint(cvmGet(projectedZ, 0, 0), cvmGet(projectedZ, 1, 0)), CV_RGB(0, 0, 255), 3, 8);

				cvShowImage("MarkerTrackerTest", display);
				live_show = true;

				int pressed_key = cvWaitKey(10);
				if (pressed_key == ' ')
				{
					int p[3];
					p[0] = CV_IMWRITE_JPEG_QUALITY;
					p[1] = 95;
					p[2] = 0;
					cvSaveImage((m_options.image_path + image_name).c_str(), live_video, p);
					image_counter++;
					LOG(INFO) << std::endl;

					// R|T matrix
					cv::Mat RT_mat(3, 4, CV_64FC1);

					//first column of rotation
					RT_mat.at<double>(0, 0) = modelview_matrices[0 + 16 * i];
					RT_mat.at<double>(1, 0) = modelview_matrices[1 + 16 * i];
					RT_mat.at<double>(2, 0) = modelview_matrices[2 + 16 * i];

					//second column of rotation
					RT_mat.at<double>(0, 1) = modelview_matrices[4 + 16 * i];
					RT_mat.at<double>(1, 1) = modelview_matrices[5 + 16 * i];
					RT_mat.at<double>(2, 1) = modelview_matrices[6 + 16 * i];

					//third column of rotation
					RT_mat.at<double>(0, 2) = modelview_matrices[8 + 16 * i];
					RT_mat.at<double>(1, 2) = modelview_matrices[9 + 16 * i];
					RT_mat.at<double>(2, 2) = modelview_matrices[10 + 16 * i];

					//translation
					RT_mat.at<double>(0, 3) = modelview_matrices[12 + 16 * i];
					RT_mat.at<double>(1, 3) = modelview_matrices[13 + 16 * i];
					RT_mat.at<double>(2, 3) = modelview_matrices[14 + 16 * i];
					LOG(INFO) << std::endl;

					ImageInfo info;
					info.image_name = image_name;
					info.image_width = live_video->width;
					info.image_height = live_video->height;
					info.K_mat = K_mat;
					info.RT_mat = RT_mat;
					infos.emplace_back(info);

					LOG(INFO) << std::endl;
				}
				
				LOG(INFO) << "Succeed to estimate camera pose of image " << image_name << std::endl;
			}

			if (!live_show)
			{
				cvShowImage("MarkerTrackerTest", live_video);
			}
			
		}

	}

	WriteImageInfoToFile(m_options.image_info_file_path, infos);

	delete[] marker_visible_flags;
	delete[] modelview_matrices;

	cvReleaseMat(&Kmat);
	cvReleaseMat(&Rmat);
	cvReleaseMat(&tvec);
	cvReleaseMat(&point0);
	cvReleaseMat(&pointX);
	cvReleaseMat(&pointY);
	cvReleaseMat(&pointZ);
	cvReleaseMat(&projected0);
	cvReleaseMat(&projectedX);
	cvReleaseMat(&projectedY);
	cvReleaseMat(&projectedZ);

	LOG(INFO) << "Succeed to esitmate all camera poses" << std::endl;

	return true;
}
