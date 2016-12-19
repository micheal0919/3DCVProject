#include "ImageInfoIO.h"

#include <opencv2/opencv.hpp>
#include <glog/logging.h>
#include <tinyxml.h>

bool WriteImageInfoToFile(const std::string& file_path, const std::vector<ImageInfo>& infos)
{
	TiXmlDocument* myDocument = new TiXmlDocument();
	TiXmlDeclaration* decl = new TiXmlDeclaration("1.0", "UTF-8", "");
	myDocument->LinkEndChild(decl);

	TiXmlElement* RootElement = new TiXmlElement("ImageInfos");
	myDocument->LinkEndChild(RootElement);

	// make double more prcise
	const int magnitude = 1;
	RootElement->SetAttribute("magnitude", magnitude);

	for (size_t i = 0; i < infos.size(); i++)
	{
		TiXmlElement* info = new TiXmlElement("ImageInfo");
		RootElement->LinkEndChild(info);

		// set file name
		TiXmlElement* name = new TiXmlElement("image_name");
		name->LinkEndChild(new TiXmlText(infos[i].image_name.c_str()));
		info->LinkEndChild(name);

		// set image size
		info->SetAttribute("width", infos[i].image_width);
		info->SetAttribute("height", infos[i].image_height);

		// set k matrix
		TiXmlElement* k_mat = new TiXmlElement("k_mat");
		info->LinkEndChild(k_mat);
		cv::Mat K_mat = infos[i].K_mat * magnitude;

		k_mat->SetDoubleAttribute("k1", K_mat.at<double>(0, 0));
		k_mat->SetDoubleAttribute("k2", K_mat.at<double>(0, 1));
		k_mat->SetDoubleAttribute("k3", K_mat.at<double>(0, 2));

		k_mat->SetDoubleAttribute("k4", K_mat.at<double>(1, 0));
		k_mat->SetDoubleAttribute("k5", K_mat.at<double>(1, 1));
		k_mat->SetDoubleAttribute("k6", K_mat.at<double>(1, 2));

		k_mat->SetDoubleAttribute("k7", K_mat.at<double>(2, 0));
		k_mat->SetDoubleAttribute("k8", K_mat.at<double>(2, 1));
		k_mat->SetDoubleAttribute("k9", K_mat.at<double>(2, 2));

		// set rt matrix
		TiXmlElement* rt_mat = new TiXmlElement("rt_mat");
		info->LinkEndChild(rt_mat);
		cv::Mat RT_mat = infos[i].RT_mat * magnitude;

		rt_mat->SetDoubleAttribute("rt1", RT_mat.at<double>(0, 0));
		rt_mat->SetDoubleAttribute("rt2", RT_mat.at<double>(0, 1));
		rt_mat->SetDoubleAttribute("rt3", RT_mat.at<double>(0, 2));
		rt_mat->SetDoubleAttribute("rt4", RT_mat.at<double>(0, 3));

		rt_mat->SetDoubleAttribute("rt5", RT_mat.at<double>(1, 0));
		rt_mat->SetDoubleAttribute("rt6", RT_mat.at<double>(1, 1));
		rt_mat->SetDoubleAttribute("rt7", RT_mat.at<double>(1, 2));
		rt_mat->SetDoubleAttribute("rt8", RT_mat.at<double>(1, 3));

		rt_mat->SetDoubleAttribute("rt9", RT_mat.at<double>(2, 0));
		rt_mat->SetDoubleAttribute("rt10", RT_mat.at<double>(2, 1));
		rt_mat->SetDoubleAttribute("rt11", RT_mat.at<double>(2, 2));
		rt_mat->SetDoubleAttribute("rt12", RT_mat.at<double>(2, 3));

	}

	myDocument->SaveFile(file_path.c_str());

	return true;
}

bool ReadImageInfoFromFile(const std::string& file_path, std::vector<ImageInfo>& infos)
{
	// open xml file
	TiXmlDocument* myDocument = new TiXmlDocument(file_path.c_str());
	CHECK(myDocument->LoadFile()) << "Fail to load image infos xml file from disk";

	// load root node
	TiXmlElement* RootElement = myDocument->RootElement();
	int magnitude;
	RootElement->QueryIntAttribute("magnitude", &magnitude);
	TiXmlHandle rootHandle(RootElement);

	// get image info from subnodes iteratively
	infos.clear();
	TiXmlElement* pElement = NULL;
	for (pElement = rootHandle.FirstChildElement("ImageInfo").Element(); pElement; pElement = pElement->NextSiblingElement())
	{
		ImageInfo info;
		int width;
		pElement->QueryIntAttribute("width", &width);
		info.image_width = width;

		int height;
		pElement->QueryIntAttribute("height", &height);
		info.image_height = height;

		std::string image_name = pElement->FirstChild("image_name")->FirstChild()->Value();
		info.image_name = image_name;

		// get K matrix
		cv::Mat K_mat(3, 3, CV_64FC1);
		pElement->FirstChild("k_mat")->ToElement()->QueryDoubleAttribute("k1", &K_mat.at<double>(0, 0));
		pElement->FirstChild("k_mat")->ToElement()->QueryDoubleAttribute("k2", &K_mat.at<double>(0, 1));
		pElement->FirstChild("k_mat")->ToElement()->QueryDoubleAttribute("k3", &K_mat.at<double>(0, 2));

		pElement->FirstChild("k_mat")->ToElement()->QueryDoubleAttribute("k4", &K_mat.at<double>(1, 0));
		pElement->FirstChild("k_mat")->ToElement()->QueryDoubleAttribute("k5", &K_mat.at<double>(1, 1));
		pElement->FirstChild("k_mat")->ToElement()->QueryDoubleAttribute("k6", &K_mat.at<double>(1, 2));

		pElement->FirstChild("k_mat")->ToElement()->QueryDoubleAttribute("k7", &K_mat.at<double>(2, 0));
		pElement->FirstChild("k_mat")->ToElement()->QueryDoubleAttribute("k8", &K_mat.at<double>(2, 1));
		pElement->FirstChild("k_mat")->ToElement()->QueryDoubleAttribute("k9", &K_mat.at<double>(2, 2));

		info.K_mat = K_mat / magnitude;

		// get rotation and translation matrix
		cv::Mat RT_mat(3, 4, CV_64FC1);
		pElement->FirstChild("rt_mat")->ToElement()->QueryDoubleAttribute("rt1", &RT_mat.at<double>(0, 0));
		pElement->FirstChild("rt_mat")->ToElement()->QueryDoubleAttribute("rt2", &RT_mat.at<double>(0, 1));
		pElement->FirstChild("rt_mat")->ToElement()->QueryDoubleAttribute("rt3", &RT_mat.at<double>(0, 2));
		pElement->FirstChild("rt_mat")->ToElement()->QueryDoubleAttribute("rt4", &RT_mat.at<double>(0, 3));

		pElement->FirstChild("rt_mat")->ToElement()->QueryDoubleAttribute("rt5", &RT_mat.at<double>(1, 0));
		pElement->FirstChild("rt_mat")->ToElement()->QueryDoubleAttribute("rt6", &RT_mat.at<double>(1, 1));
		pElement->FirstChild("rt_mat")->ToElement()->QueryDoubleAttribute("rt7", &RT_mat.at<double>(1, 2));
		pElement->FirstChild("rt_mat")->ToElement()->QueryDoubleAttribute("rt8", &RT_mat.at<double>(1, 3));

		pElement->FirstChild("rt_mat")->ToElement()->QueryDoubleAttribute("rt9", &RT_mat.at<double>(2, 0));
		pElement->FirstChild("rt_mat")->ToElement()->QueryDoubleAttribute("rt10", &RT_mat.at<double>(2, 1));
		pElement->FirstChild("rt_mat")->ToElement()->QueryDoubleAttribute("rt11", &RT_mat.at<double>(2, 2));
		pElement->FirstChild("rt_mat")->ToElement()->QueryDoubleAttribute("rt12", &RT_mat.at<double>(2, 3));

		info.RT_mat = RT_mat / magnitude;

		// put info into outpu vector
		infos.emplace_back(info);

	}
	return true;
}

