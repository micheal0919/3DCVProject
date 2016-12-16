//********************************
// MarkerTracker
// (c) 2010 DFKI / AV
//********************************
#include "MarkerTrackerInterface.h"
#include "MarkerWorld.h"
#include "opencv\highgui.h"

#include <fstream>
#include <iostream>
#include <windows.h>
#include <vector>
#include "MarkerTracker.h"
#include "MatrixCv.h"
#include <iomanip>


class MarkerTrackerInterfaceImpl
{
public:
    MarkerTrackerInterfaceImpl():
    m_width(0),
    m_height(0),
    m_depth(0),
    m_channelCount(0),
    m_rigidBodyCount(0),
    m_forceZup(true),
    m_debugWindow(true),
    m_started(false),
    m_live(0),
    m_remap_temp(0),
    m_mapx(0),
    m_mapy(0)
    {
    }

    ~MarkerTrackerInterfaceImpl()
    {
        if(m_live!=0)
        {
            cvReleaseImageHeader(&m_live);
        }
        if(m_remap_temp!=0)
        {
            cvReleaseImage(&m_remap_temp);
        }
        if(m_mapx!=0)
        {
            cvReleaseImage(&m_mapx);
        }
        if(m_mapy!=0)
        {
            cvReleaseImage(&m_mapy);
        }
    }

    unsigned int m_width;
    unsigned int m_height;
    unsigned int m_depth;
    unsigned int m_channelCount;
    unsigned int m_rigidBodyCount;
    bool m_forceZup;
    bool m_debugWindow;
    bool m_started;
    IplImage* m_live;
    IplImage* m_remap_temp;
    IplImage* m_mapx;
    IplImage* m_mapy;
    VectorCv5 m_distortion_coeffs;
    IntrinsicMatrix m_K;
    IntrinsicMatrix m_Kt;
    IntrinsicMatrix m_Kinv;
    MarkerWorld m_world;
    MarkerTracker m_tracker;

};

MarkerTrackerInterface::MarkerTrackerInterface():_impl(0)
{
    _impl = new MarkerTrackerInterfaceImpl;
	
    //initialize values;
    _im_num = 510;
    
}


MarkerTrackerInterface::~MarkerTrackerInterface()
{
    if(_impl)
    {

        //destroy impl
        
        //delete
        delete _impl;
    }
}

bool MarkerTrackerInterface::start(const std::string& intrinsics_file, const std::string& world_file,
           const bool debugWindow, const bool forceZup,
           unsigned int width, unsigned int height,
           unsigned int depth, unsigned int channel_count,
           std::string& error_string)
{

    _impl->m_height = height;
    _impl->m_width = width;
    _impl->m_depth = depth;
    _impl->m_channelCount = channel_count;
    _impl->m_forceZup = forceZup;
    _impl->m_debugWindow = debugWindow;

    _impl->m_live = cvCreateImageHeader(cvSize(width, height), depth, channel_count);
    _impl->m_remap_temp = cvCreateImage(cvSize(width, height), depth, channel_count);
    _impl->m_mapx = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 1);
    _impl->m_mapy = cvCreateImage(cvSize(width, height), IPL_DEPTH_32F, 1);

    //read intrinsics
    double cx, cy ,fx,fy,k1,k2,k3,k4,k5;
    std::ifstream infile(intrinsics_file.c_str());

    if(!infile.is_open())
    {
        error_string =  std::string("Camera parameters file not found. Please checkt input: '") + intrinsics_file+std::string("'");
        return false;
    }

    infile >> cx >> cy >> fx >> fy >> k1 >> k2 >> k3 >> k4 >> k5;
    infile.close();

    cvmSet(_impl->m_K.cvMatPtr(), 0,0, fx*width);
    cvmSet(_impl->m_K.cvMatPtr(), 0,1, 0);
    cvmSet(_impl->m_K.cvMatPtr(), 0,2, cx*width);
    cvmSet(_impl->m_K.cvMatPtr(), 1,0, 0);
    cvmSet(_impl->m_K.cvMatPtr(), 1,1, fy*height);
    cvmSet(_impl->m_K.cvMatPtr(), 1,2, cy*height);
    cvmSet(_impl->m_K.cvMatPtr(), 2,0, 0);
    cvmSet(_impl->m_K.cvMatPtr(), 2,1, 0);
    cvmSet(_impl->m_K.cvMatPtr(), 2,2, 1);

    //precompute transpose and inverse
    cvTranspose(_impl->m_K.cvMatPtr(), _impl->m_Kt.cvMatPtr());
    cvInv(_impl->m_K.cvMatPtr(), _impl->m_Kinv.cvMatPtr());

    cvmSet(_impl->m_distortion_coeffs.cvMatPtr(),0,0,k1);
    cvmSet(_impl->m_distortion_coeffs.cvMatPtr(),1,0,k2);
    cvmSet(_impl->m_distortion_coeffs.cvMatPtr(),2,0,k3);
    cvmSet(_impl->m_distortion_coeffs.cvMatPtr(),3,0,k4);
    cvmSet(_impl->m_distortion_coeffs.cvMatPtr(),4,0,k5);

    cvInitUndistortMap(_impl->m_K.cvMatPtr(), _impl->m_distortion_coeffs.cvMatPtr(), _impl->m_mapx, _impl->m_mapy);

	cv::Mat cppmapx = cv::cvarrToMat(_impl->m_mapx);
	cv::Mat cppmapy = cv::cvarrToMat(_impl->m_mapy);

	if (cppmapx.size().area() <= 0 || cppmapy.size().area() <= 0)
	{
		error_string = std::string("The distortion maps could not be computed correctly - check calibration file");
		return false;
	}


    //read world and count number of rigid bodies
    std::ifstream world_stream;
    world_stream.open(world_file.c_str(), std::ifstream::in);
    if(!world_stream.is_open() || !world_stream.good())
    {
        error_string =  std::string("Could not find/open marker world file: '") + world_file+std::string("'");
        return false;
    }
    if(!_impl->m_world.read(world_stream))
    {
        error_string =  std::string("Could not read marker world file: '") + world_file+std::string("'");
        return false;
        
    }

    unsigned int marker_count=0;
    for(MarkerWorld::MarkerPtrIterator it=_impl->m_world.markerBegin();
        it!=_impl->m_world.markerEnd();
        it++)
    {
        marker_count++;
    }
    std::cout << "Read world with " << _impl->m_world.size() << " rigid bodies and a total of " << marker_count << " markers" << std::endl;
    _impl->m_rigidBodyCount = _impl->m_world.size();
    
    //init marker tracker
    _impl->m_tracker.init(cvSize(width, height));

    if(debugWindow)
    {
        cvNamedWindow("MarkerTracker");
    }

    _impl->m_started = true;
    return true;
}

void MarkerTrackerInterface::getData(bool* markerVisible, double* modelviewMatrix, unsigned char* videoData)
{
    _impl->m_live->imageData = (char*) videoData;
    MarkerTracker& tracker = _impl->m_tracker;

    cvScale(_impl->m_live, _impl->m_remap_temp);
	cv::Mat cppmapx = cv::cvarrToMat(_impl->m_mapx);
	cv::Mat cppmapy = cv::cvarrToMat(_impl->m_mapy);

	if (cppmapx.size().area() <= 0 || cppmapy.size().area() <= 0)
	{
		std::cout << "will break" << std::endl;
		return;
	}
	cvRemap(_impl->m_remap_temp, _impl->m_live, _impl->m_mapx, _impl->m_mapy);
	
	//set the gray image
    tracker.setImage(_impl->m_live, _impl->m_debugWindow);

    //enhance constrast
    //tracker.enhanceConstrast();

    //find canny edges
    tracker.doCanny();

    //find closed contours
    tracker.findClosedContours();

    //find ellipses
    tracker.findEllipses();

    //ellipses to marker observations
    tracker.ellipsesToMarkerObservations(_impl->m_K,_impl->m_Kt, _impl->m_forceZup);

    //tracker.showMarkerObservations( _impl->m_K);

    //sample marker observations
    tracker.sampleMarkerObservations(_impl->m_K);

    //compute correlations
    tracker.computeCorrelations(_impl->m_world);

    //compute assignement
    tracker.computeAssignment();


    //compute pose
    tracker.computeAllRigidBodyPoses(_impl->m_world);


    //write result
    tracker.writeData(_impl->m_world, markerVisible, modelviewMatrix);



    tracker.showMarkerAssignments( _impl->m_K, _impl->m_world);

    tracker.showPose( _impl->m_K, _impl->m_world);


    if(_impl->m_debugWindow)
    {
        cvShowImage("MarkerTracker", tracker.getDebugImage());
        //8 digit zero padded image number
        //std::stringstream ss_8dzp_imnum;
        //ss_8dzp_imnum << std::setw(8) << std::setfill('0') << _im_num++;

        //cvSaveImage((std::string("../../data/data/seq_save/")+ss_8dzp_imnum.str()+std::string(".jpg")).c_str(),tracker.getDebugImage());
        cvWaitKey(10);
    }
	return;
}

unsigned int MarkerTrackerInterface::getRigidBodyCount()
{

    return _impl->m_rigidBodyCount;
}

void MarkerTrackerInterface::getKMatrix(double * K_matrix)
{
    if(!_impl->m_started) return;

    for(unsigned int i=0; i<3; i++)
    {
        for(unsigned int j=0; j<3; j++)
        {
            K_matrix[i+j*3] =  cvmGet(_impl->m_K.cvMatPtr(),i,j);
        }
    }
}

void MarkerTrackerInterface::getProjectionMatrix(double * projection_matrix)
{
	
    if(!_impl->m_started) return;

	double fx =  cvmGet(_impl->m_K.cvMatPtr(),0,0);
	double fy =  cvmGet(_impl->m_K.cvMatPtr(),1,1);
	double cx =  cvmGet(_impl->m_K.cvMatPtr(),0,2);
	double cy =  cvmGet(_impl->m_K.cvMatPtr(),1,2);

	double nearFact = 0.1;
	double farFact = 10000;

	int width = _impl->m_width;
	int height = _impl->m_height;
    double gl_cy = _impl->m_height - cy;
	double left   = (-cx*nearFact)/fx;
	double right  = ((_impl->m_width - cx)*nearFact)/fx;
	double bottom = -(gl_cy*nearFact)/fy;
	double top    = ((_impl->m_height - gl_cy)*nearFact)/fy;

	projection_matrix[0] = 2*nearFact/(right-left);
	projection_matrix[1] = 0.0;
	projection_matrix[2] = 0.0;
	projection_matrix[3] = 0.0;
	projection_matrix[4] = 0.0;
	projection_matrix[5] = 2*nearFact/(top-bottom);
	projection_matrix[6] = 0.0;
	projection_matrix[7] = 0.0;
	projection_matrix[8] = (right+left)/(right-left);
	projection_matrix[9] = (top+bottom)/(top-bottom);
	projection_matrix[10] = -(farFact+nearFact)/(farFact-nearFact);
	projection_matrix[11] = -1.0;
	projection_matrix[12] = 0.0;
	projection_matrix[13] = 0.0;
	projection_matrix[14] = -2*nearFact*farFact/(farFact-nearFact);
	projection_matrix[15] = 0.0;

	
	return;
}

