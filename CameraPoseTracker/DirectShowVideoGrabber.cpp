//********************************
// DirectShowVideoGrabber
// (c) 2010 DFKI / AV
//********************************
#include "DirectShowVideoGrabber.h"
#include "DSVL.h"
#include "opencv/cv.h"
#include <iostream>


static DSVL_VideoSource   *gVid = NULL;
static MemoryBufferHandle *mbuf_handle = NULL;

DirectShowVideoGrabber::DirectShowVideoGrabber():
m_image(0),
m_width(0),
m_height(0),
m_bufferCheckedOut(false)
{

}

DirectShowVideoGrabber::~DirectShowVideoGrabber()
{

}

bool DirectShowVideoGrabber::init(char* config)
{
    long w,h;
    
    if (gVid != NULL) {
        fprintf(stderr, "videoOpen(): Error, device is already open.\n");
        return false;
    }

    char config_default[] = "<?xml version=\"1.0\" encoding=\"UTF-8\"?><dsvl_input><camera show_format_dialog=\"true\" friendly_name=\"\"><pixel_format><RGB32 flip_h=\"false\" flip_v=\"true\"/></pixel_format></camera></dsvl_input>";


    CoInitialize(NULL);

    // Allocate the parameters structure and fill it in.
    gVid = new DSVL_VideoSource();
    mbuf_handle = new MemoryBufferHandle;
    if (!config) 
    {
        if (FAILED(gVid->BuildGraphFromXMLString(config_default))) return false;
    } 
    else 
    {
        if (strncmp(config, "<?xml", 5) == 0) {
            if (FAILED(gVid->BuildGraphFromXMLString(config))) return false;
        } else {
            if (FAILED(gVid->BuildGraphFromXMLFile(config))) return false;
        }
    }
    if (FAILED(gVid->EnableMemoryBuffer())) return false;

    if (gVid == NULL) return  false;

    gVid->GetCurrentMediaFormat(&w, &h,NULL,NULL);
	

    //m_image = cvCreateImageHeader(cvSize((int)w,(int)h), IPL_DEPTH_8U, 4);
    m_image = cvCreateImage(cvSize((int)w,(int)h), IPL_DEPTH_8U, 4);

    m_width=(unsigned int)w;
    m_height=(unsigned int)h;

    gVid->Run();

    return true;

}

IplImage* DirectShowVideoGrabber::getImage()
{
    unsigned char * videoDataPtr = NULL;
    DWORD wait_result;

    while( videoDataPtr == NULL ) 
    {
        if (m_bufferCheckedOut)
        {
            gVid->CheckinMemoryBuffer(*mbuf_handle);
            m_bufferCheckedOut = false;
        }
        wait_result = gVid->WaitForNextSample(0L);
        if (wait_result == WAIT_OBJECT_0)
        {
            gVid->CheckoutMemoryBuffer(mbuf_handle, &videoDataPtr);
            m_bufferCheckedOut = true;
        }
        Sleep(2);
    }

    //m_image->imageData = (char*)(videoDataPtr);
    memcpy(m_image->imageData, videoDataPtr, m_width * m_height * m_image->nChannels * sizeof(uchar));
    
    //next image
    if (m_bufferCheckedOut) 
    {
        gVid->CheckinMemoryBuffer(*mbuf_handle, true);
        m_bufferCheckedOut = false;
    }

    return m_image;

}

void DirectShowVideoGrabber::close()
{
    if (m_bufferCheckedOut)
    {
        gVid->CheckinMemoryBuffer(*mbuf_handle, true);
        m_bufferCheckedOut = false;
    }
    gVid->Stop();
    delete gVid;
    delete mbuf_handle;
    mbuf_handle = NULL;
    gVid = NULL;
    CoUninitialize();

    cvReleaseImage(&m_image);

   
}











