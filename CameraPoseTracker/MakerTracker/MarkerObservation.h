#pragma once
#include "Ellipse2D.h"
#include "opencv\cxcore.h"
#include "MatrixCv.h"

class MarkerObservation
{
public:
    
    void setNormalizedConic(const ConicMatrix& conicMatrix);
    void setLocalPoseHypotheses(const PoseMatrix& P1, const PoseMatrix& P2);
    PoseMatrix getP(int hypothesis) const {return (hypothesis==1?_P1:_P2);}
    ConicMatrix getNormalizedConic() const {return _normalizedConic;}
    int getContourLength() const {return _contourLength;}
    void setContourLength(int length) {_contourLength = length;}
    Ellipse2D* getEllisePtr() const {return _ellipse;}
    void setEllisePtr(Ellipse2D* ell) {_ellipse = ell;}

private:

    ConicMatrix    _normalizedConic; //< conic matrix in K-normalized image
    PoseMatrix    _P1; //< first local pose hypothesis
    PoseMatrix    _P2; //< second local pose hypothesis
    int        _contourLength; //< length of the generating contour
    Ellipse2D * _ellipse;
   

};
