//********************************
// MarkerTracker
// (c) 2015 DFKI / AV
//********************************
#pragma once
#include <string>
#include <vector>



class MarkerTrackerInterfaceImpl;


/*
class MarkerTrackerInterface

This class defines the interface for the marker tracker with three types
of methods:
1. start():
start is called once to initialize the camera and the markerTracker.
- intrinsics_file is the file containing the calibration of the camera
- world_file is the file containing the definition of the marker world
- debugWindow (optional) provides a viewer with the image of the camera
and a simple augmentation.
2. Following methods can be called once start() has been called (otherwise,
the result will be unchanged (getProjectionMatrix, getKMatrix):
- getKMatrix returns a 3x3 matrix representing the K matrix
- getProjectionmMatrix returns a 4x4 matrix representing the projection
parameters to use in OpenGL
- getRigidBodyCount() returns the number of independent rigid bodies that are tracked.
3. getData():
getData() can be called at any time once start() has been called. It
returns 3 values:
- markerVisible tells whether the marker has been found in the image or
not.
- modelviewMatrix (optional) returns a 4x4 matrix representing the 
position of the camera in relation to the world coordinate system.
- videoData (optional) should be a pointer to a buffer where the camera
image is copied. The size of the buffer has to be as least the value
returned by bufferSize in the start() method.
*/

class MarkerTrackerInterface
{
public:

    MarkerTrackerInterface();
	~MarkerTrackerInterface();

    bool start(const std::string& intrinsics_file, const std::string& world_file,
        const bool debugWindow, const bool forceZup,
        unsigned int width, unsigned int height,
        unsigned int depth, unsigned int channel_count,
        std::string& error_string);

	void getData(bool * markerVisible, double * modelviewMatrix=0, unsigned char * videoData=0);
    unsigned int getRigidBodyCount();
    void getProjectionMatrix(double * projection_matrix);
    void getKMatrix(double * K_matrix);
	
    
	
private:
	
    MarkerTrackerInterfaceImpl * _impl;
    unsigned int _im_num;
    
};