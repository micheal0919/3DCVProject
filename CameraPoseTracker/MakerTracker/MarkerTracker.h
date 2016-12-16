#pragma once

#include <opencv\cv.h>
#include "MarkerObservationSamplingMap.h"
#include "MarkerCorrelationTable.h"
#include "MarkerCorrespondencyMap.h"
#include "MarkerRigidBodyPoseMap.h"

class ClosedContour;
class Ellipse2D;
class MarkerObservation;
class MarkerWorld;
class MarkerModel;
class MarkerSampling;



class MarkerTracker
{
public:
    MarkerTracker();
    ~MarkerTracker();

    void init(const CvSize& s );

    void setImage(IplImage* im, bool with_debug_image =false);
    IplImage* getDebugImage() const;
    void enhanceConstrast();
    void doCanny();
    void findClosedContours();
    void findEllipses();
    void ellipsesToMarkerObservations(const IntrinsicMatrix& K, const IntrinsicMatrix& Kt, bool forceZup);
    void sampleMarkerObservations(const IntrinsicMatrix& K);
    void computeCorrelations(const MarkerWorld& world);
    void computeAssignment();
    void computeAllRigidBodyPoses(const MarkerWorld& world);
    void writeData(const MarkerWorld& world, bool* markerVisible, double* modelviewMatrix);

    //debug
    void showMarkerObservations(const IntrinsicMatrix& K);
    void showMarkerAssignments(const IntrinsicMatrix& K , const MarkerWorld& world);
    void showPose(const IntrinsicMatrix& K , const MarkerWorld& world);
     
    IntrinsicMatrix Kbla;
private:
    //pointer to an image
    IplImage* _live;

    //images owned by MarkerTracker
    IplImage* _grey;
    IplImage* _canny;
    IplImage* _tmp;
    IplImage* _debug;

    unsigned char _logTable[256];
    CvMemStorage* _contourMemStorage;
    std::vector<ClosedContour> _contours;
    std::vector<Ellipse2D> _ellipses;
    std::vector<MarkerObservation> _markerObservations;
    MarkerObservationSamplingMap _markerObservationSamplingMap;
    MarkerCorrelationTable _markerCorrelationTable;
    MarkerCorrespondencyMap _markerCorrespondencyMap;
    MarkerRigidBodyPoseMap _markerRigidBodyPoseMap;


    double fitEllipseMatrix( const CvPoint* inpoints, int numContourPoints, ConicMatrix& outConic );
    void computeChenHypotheses( const ConicMatrix& normalizedConic, PoseMatrix& P1, PoseMatrix& P2, bool forceZup, const IntrinsicMatrix& K );
    void eigenDecomp(CvMat* iC, CvMat* oDiag, CvMat* oEigenVec);
    void sortEigenDecomp(CvMat* eigenVecs, CvMat* eigenVals);
    void drawPatch( IplImage* im, const PoseMatrix& P, const IntrinsicMatrix& K, CvScalar color );
    bool getGreyValueAtProjectedPoint( double x, double y, const PoseMatrix& P, const IntrinsicMatrix& K, double& val );
    bool project3DPointToPixel(const IntrinsicMatrix& K, const PoseMatrix& P, double X, double Y, double Z, double& x, double& y) const;
    void computeBestAngularCorrelation( const MarkerModel& model, const MarkerSampling& sampling, double& correl, double &angle, int& hypothesis );
    double crossCorrelation( int offset, int resolution, const int * model, const double * samplesVec, int size );

    //compute the pose of a single rigid body
    bool computeSingleRigidBodyPose(const MarkerRigidBody* mrb, PoseMatrix& pose_data);
    
    //compute homography for a plane containing one or several markers
    Homography computePlanarHomography(const MarkerPlane& plane, const std::vector<MarkerCorrespondencyMap::const_iterator>& visibleMarkers) const;
    //compute homography for a plane containing one single marker
    Homography computePlanarHomography1(const MarkerPlane& plane, const std::vector<MarkerCorrespondencyMap::const_iterator>& visibleMarkers) const;
    //compute homography for a plane containing two markers
    Homography computePlanarHomography2(const MarkerPlane& plane, const std::vector<MarkerCorrespondencyMap::const_iterator>& visibleMarkers) const;
    //compute homography for a plane containing 3 or more markers
    Homography computePlanarHomography3orMore(const MarkerPlane& plane, const std::vector<MarkerCorrespondencyMap::const_iterator>& visibleMarkers) const;
    
    //computation of rigid body pose from a set of planar homographies
    PoseMatrix computePoseFromPlanes( const std::vector<std::pair<const MarkerPlane*, Homography> >& visiblePlanes ) const;
    //single plane
    PoseMatrix computePoseFromPlanes1( const std::vector<std::pair<const MarkerPlane*, Homography> >& visiblePlanes ) const;
    void planeHomographyToPose( const Homography& H, MatrixCv3x3& R, VectorCv3& t) const;

};