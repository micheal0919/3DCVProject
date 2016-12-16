//********************************
// DirectShowVideoGrabber
// (c) 2010 DFKI / AV
//********************************
#pragma once

typedef struct _IplImage IplImage;
class DSVL_VideoSource;



/**
DirectShowVideoGrabber is an simple interface to DSVL
*/
class DirectShowVideoGrabber
{
public:
    DirectShowVideoGrabber();
    ~DirectShowVideoGrabber();

    /*! initialization with optional xml configuration */
    bool init(char* config=0);

    /*! width of image once initialized */
    unsigned int width()const {return m_width;} 

    /*! height of image once initialized */
    unsigned int height()const {return m_height;} 

    /*! get current image from camera (once initialized) */
    IplImage* getImage();

    /*! closes the camera */
    void close();

protected:

    IplImage *		m_image;
    unsigned int    m_width;
    unsigned int    m_height;
    bool            m_bufferCheckedOut;


    
};