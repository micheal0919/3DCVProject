#include "ClosedContour.h"
#include "opencv\cv.h"

using namespace std;

ClosedContour::ClosedContour(void):
_pointArraySize(0),
_pointArray(0)
{
}

ClosedContour::~ClosedContour(void)
{
    if(_pointArray)
    {
        free(_pointArray);
    }

}

ClosedContour::ClosedContour(const ClosedContour& source)
    : _pointArraySize(0), _pointArray(0)
{
    resize(source._pointArraySize);
    memcpy(_pointArray,source._pointArray, _pointArraySize*sizeof(CvPoint));
}
ClosedContour& ClosedContour::operator =(const ClosedContour&source)
{
    resize(source._pointArraySize);
    memcpy(_pointArray,source._pointArray, _pointArraySize*sizeof(CvPoint));
    return *this;
}


void ClosedContour::resize(unsigned int size)
{
    if(_pointArray)
    {
        free(_pointArray);
    }

    _pointArraySize = size;

    _pointArray = (CvPoint*)malloc( _pointArraySize*sizeof(CvPoint) );

}

int ClosedContour::size() const
{
    return _pointArraySize;
}


