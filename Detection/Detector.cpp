#include "Detector.h"
#include "Detector.h"

#include "glog/logging.h"

CDetector::CDetector(const CDetector::Options& options)
:m_options(options)
{
}


CDetector::~CDetector()
{
}

bool CDetector::Detect()
{
	// check if we succeeded
	if (!m_cap.init())
	{
		LOG(ERROR) << "Fail to open video capture" << std::endl;
		return false;
	}

	while (true)
	{
		IplImage * live_video = m_cap.getImage();
		LOG(INFO) << "Image size " << live_video->width << "x" << live_video->height << ", " << live_video->nChannels << " channels/pixel" << ", " << live_video->depth << " bits/channel" << std::endl;
		cvShowImage("MarkerTrackerTest", live_video);
	}

	return true;
}

bool CDetector::ReadYmlFile()
{
	return true;
}
