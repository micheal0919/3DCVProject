#pragma once

struct CvPoint;

class ClosedContour 
{
public:
    ClosedContour(void);
    ~ClosedContour(void);

    ClosedContour(const ClosedContour& source);
    ClosedContour& operator =(const ClosedContour&source);
	
    CvPoint * getPointArray() const {return _pointArray;}
    void resize(unsigned int size);
    int size() const;
	
protected:
    int         _pointArraySize;
    CvPoint*    _pointArray;
	
};
