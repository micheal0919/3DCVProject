#pragma once
#include "opencv\cv.h"
#include "MatrixCv.h"


class Ellipse2D
{
public:

    Ellipse2D();
    Ellipse2D(const ConicMatrix& conic);
	
	void convertConicToDrawingParameters( CvPoint& center, CvSize& size, double& angle ) const;
    ConicMatrix getConic() const;
    void setNumPoints(int numPoints);
    int getNumPoints() const;
    void setConic(const ConicMatrix& conic);

    

private:
    ConicMatrix _conic;
    int _numPoints;
};

