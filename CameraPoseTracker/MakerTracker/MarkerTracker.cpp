#include "MarkerTracker.h"
#include <iostream>
#include "ClosedContour.h"
#include "Ellipse2D.h"
#include "MarkerObservation.h"
#include "MarkerWorld.h"

MarkerTracker::MarkerTracker() :_grey(0), _live(0), _canny(0), _tmp(0), _debug(0)
{
    for(int i = 0; i < 256; i++) _logTable[i] = (unsigned char)(105.886*log10((float)i+1)+0.5);
    _contourMemStorage = cvCreateMemStorage(0);

}

MarkerTracker::~MarkerTracker()
{
    if(_grey)
    {
        cvReleaseImage(&_grey);
    }
    if(_canny)
    {
        cvReleaseImage(&_canny);
    }
    if(_tmp)
    {
        cvReleaseImage(&_tmp);
    }
    if(_debug)
    {
        cvReleaseImage(&_debug);
    }
    
}

void MarkerTracker::init( const CvSize& s )
{
    if(_grey)
    {
        cvReleaseImage(&_grey);
        _grey=0;
    }
    if(_canny)
    {
        cvReleaseImage(&_canny);
        _canny=0;
    }
    if(_tmp)
    {
        cvReleaseImage(&_tmp);
        _tmp=0;
    }
    if(_debug)
    {
        cvReleaseImage(&_debug);
        _debug=0;
    }
    
    //create grey image
    _grey = cvCreateImage(s,8, 1);
    _canny = cvCreateImage(s,8, 1);
    _tmp = cvCreateImage(s,8, 1);
    _debug = cvCreateImage(s,8, 3);
    


}

void MarkerTracker::setImage( IplImage* im, bool with_debug_image )
{
    if(!_grey) return;
    if(im->width!=_grey->width || im->height!=_grey->height )
    {
        std::cout << "Not same image size" << std::endl;
        return;
    }

    //copy the pointer to live
    _live = im;
    
    //make a copy of the image in grey and debug
    if(im->nChannels == 3) cvCvtColor(im, _grey, CV_BGR2GRAY);
    else if(im->nChannels == 4) cvCvtColor(im, _grey, CV_BGRA2GRAY);
    else cvScale(im, _grey);

    if(with_debug_image)
    {
        if(im->nChannels == 3)  cvScale(im, _debug);
        else if(im->nChannels == 4) cvCvtColor(im, _debug, CV_BGRA2BGR);
        else cvCvtColor(im, _debug, CV_GRAY2BGR);
    }
    


}

IplImage* MarkerTracker::getDebugImage() const
{
    return _debug;
}

void MarkerTracker::enhanceConstrast()
{
    int size = _grey->width * _grey->height;
    for(int i=0; i<size; i++)
    {
        ((uchar *)(_grey->imageData))[i]=_logTable[((uchar *)(_grey->imageData))[i]];
    }

}

void MarkerTracker::doCanny()
{
    cvScale(_grey, _tmp); //cvCanny destroys input image
    cvCanny(_tmp, _canny, 1000, 2000, 5);
}

void MarkerTracker::findClosedContours()
{
    CvSeq * contour = 0;
    cvScale(_canny, _tmp); //cvFindContours destroys input image
    cvClearMemStorage(_contourMemStorage);
    cvFindContours( _tmp, _contourMemStorage, &contour, sizeof(CvContour), CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
    _contours.clear();
    // if there is some contour at all
    if(contour)
    {
        int maxLevel = 2;
        CvTreeNodeIterator iterator;
        cvInitTreeNodeIterator( &iterator, contour, maxLevel );

        //iterate on the found contours
        while( (contour = (CvSeq*)cvNextTreeNode( &iterator )) != 0 )
        {
            //we are looking only for hole contours
            if(CV_IS_SEQ_HOLE( contour )!=true)
            {
                continue;
            }

            // Get contour point set
            _contours.push_back(ClosedContour());
            _contours.back().resize(contour->total);
            cvCvtSeqToArray(contour, _contours.back().getPointArray(), CV_WHOLE_SEQ);
        }
    }

}

void MarkerTracker::findEllipses()
{
    _ellipses.clear();

    for(unsigned int i = 0; i < _contours.size(); i++)
    {

        //we are looking only for ellipses with small fitting error
        if(_contours.at(i).size() < 50)
        {
            continue;
        }

        // Ellipse fitting (find conic) with output of the normalized error on the ellipse
        ConicMatrix fittedConic;
        double err = fitEllipseMatrix(_contours.at(i).getPointArray(), _contours.at(i).size(), fittedConic);

        //we are looking only for ellipses with small fitting error
        if(err > 100000)
        {
            continue;
        }

        //all tests passed; store found ellipse
        Ellipse2D anEllipse;
        anEllipse.setNumPoints(_contours.at(i).size());
        anEllipse.setConic(fittedConic);
        _ellipses.push_back(anEllipse);


    }


}

double MarkerTracker::fitEllipseMatrix( const CvPoint* inpoints, int numContourPoints, ConicMatrix& outConic )
{
    double bigError = 1.0e+15;
    if( numContourPoints < 6 )
    {
        std::cout << "Number of points should be >= 6"<<std::endl;
        return bigError;
    }

    /* create matrix D of  input points */
    CvMat* D = cvCreateMat( numContourPoints, 6, CV_64F );

    // fill matrix rows as (x*x, x*y, y*y, x, y, 1 )
    for(int i = 0; i < numContourPoints; i++ )
    {
        double x, y;
        double* Dptr = D->data.db + i*6;

        x = (float)inpoints[i].x;
        y = (float)inpoints[i].y;

        Dptr[0] = x * x;
        Dptr[1] = x * y;
        Dptr[2] = y * y;
        Dptr[3] = x;
        Dptr[4] = y;
        Dptr[5] = 1.;
    }


    double S[36];
    CvMat _S = cvMat(6,6,CV_64F,S);
    double C[36];
    CvMat _C = cvMat(6,6,CV_64F,C);
    double T[36];
    CvMat _T = cvMat(6,6,CV_64F,T);
    double eigenvalues[6];
    CvMat _EIGVALS = cvMat(6,1,CV_64F,eigenvalues);
    double eigenvectors[36];
    CvMat _EIGVECS = cvMat(6,6,CV_64F,eigenvectors);

    // S = D^t*D
    cvMulTransposed( D, &_S, 1 );
    cvSVD( &_S, &_EIGVALS, &_EIGVECS, 0, CV_SVD_MODIFY_A + CV_SVD_U_T );

    for(int i = 0; i < 6; i++ )
    {
        double ev = eigenvalues[i];
        ev = ev < DBL_EPSILON ? 0 : 1./sqrt(sqrt(ev));
        for(int j = 0; j < 6; j++ )
            eigenvectors[i*6 + j] *= ev;
    }

    // C = Q^-1 = transp(INVEIGV) * INVEIGV
    cvMulTransposed( &_EIGVECS, &_C, 1 );

    cvZero( &_S );
    S[2] = 2.;
    S[7] = -1.;
    S[12] = 2.;

    // S = Q^-1*S*Q^-1
    cvMatMul( &_C, &_S, &_T );
    cvMatMul( &_T, &_C, &_S );

    // and find its eigenvalues and vectors too
    //cvSVD( &_S, &_EIGVALS, &_EIGVECS, 0, CV_SVD_MODIFY_A + CV_SVD_U_T );
    cvEigenVV( &_S, &_EIGVECS, &_EIGVALS, 0 );

    int evCounter = 0;
    for( evCounter = 0; evCounter < 3; evCounter++ )
        if( eigenvalues[evCounter] > 0 )
            break;

    if( evCounter >= 3 /*eigenvalues[0] < DBL_EPSILON*/ )
    {
        return bigError;
    }

    // now find truthful eigenvector
    _EIGVECS = cvMat( 6, 1, CV_64F, eigenvectors + 6*evCounter );
    _T = cvMat( 6, 1, CV_64F, T );
    // Q^-1*eigenvecs[0]
    cvMatMul( &_C, &_EIGVECS, &_T );

    // extract vector components
    double a, b, c, d, e, f;
    a = T[0]; b = T[1]; c = T[2]; d = T[3]; e = T[4]; f = T[5];

    /*
    1) find center of ellipse
    it satisfy equation
    | a     b/2 | *  | x0 | +  | d/2 | = |0 |
    | b/2    c  |    | y0 |    | e/2 |   |0 |

    */
    double idet = a * c - b * b * 0.25;
    idet = idet > DBL_EPSILON ? 1./idet : 0;

    // we must normalize (a b c d e f ) to fit (4ac-b^2=1)
    double scale = sqrt( 0.25 * idet );

    if( scale > DBL_EPSILON )
    {
        a *= scale;
        b *= scale;
        c *= scale;
        d *= scale;
        e *= scale;
        f *= scale;
    }

    ///////////////////////////////////

    cvmSet(outConic.cvMatPtr(), 0,0, a);
    cvmSet(outConic.cvMatPtr(), 0,1, b/2.0);
    cvmSet(outConic.cvMatPtr(), 0,2, d/2.0);
    cvmSet(outConic.cvMatPtr(), 1,0, b/2.0);
    cvmSet(outConic.cvMatPtr(), 1,1, c);
    cvmSet(outConic.cvMatPtr(), 1,2, e/2.0);
    cvmSet(outConic.cvMatPtr(), 2,0, d/2.0);
    cvmSet(outConic.cvMatPtr(), 2,1, e/2.0);
    cvmSet(outConic.cvMatPtr(), 2,2, f);

    //here try to figure out the error
    //TODO: check if this analytic error is correct and / or compute geometrical error
    double err = 0;
    for(int i = 0; i < numContourPoints; i++ )
    {
        double x, y;

        x = (float)inpoints[i].x;
        y = (float)inpoints[i].y;

        err+=fabs(a*x*x+b*x*y+c*y*y+d*x+e*y+f);
    }

    err/=(numContourPoints*numContourPoints);

    if( scale <= DBL_EPSILON )
    {
        err = 1000000;
    }


    cvReleaseMat( &D );

    return err;
}

void MarkerTracker::ellipsesToMarkerObservations(const IntrinsicMatrix& K, const IntrinsicMatrix& Kt, bool forceZup)
{
    Kbla = K;
    _markerObservations.clear();
    for(unsigned int i = 0; i < _ellipses.size(); i++)
    {
        MarkerObservation mo;
        ConicMatrix normalizedConic;
        cvMatMul(Kt.cvMatPtr(), _ellipses.at(i).getConic().cvMatPtr(), normalizedConic.cvMatPtr());
        cvMatMul(normalizedConic.cvMatPtr(), K.cvMatPtr(), normalizedConic.cvMatPtr());
        //Make signature ++-
        if(cvDet(normalizedConic.cvMatPtr())>0)
        {
            cvScale(normalizedConic.cvMatPtr(), normalizedConic.cvMatPtr(), -1.0);
        }
        mo.setNormalizedConic(normalizedConic);

        PoseMatrix P1, P2;
        computeChenHypotheses(normalizedConic, P1, P2, forceZup, K);

        mo.setLocalPoseHypotheses(P1, P2);
        mo.setContourLength(_ellipses.at(i).getNumPoints());
        mo.setEllisePtr(&_ellipses.at(i));


        _markerObservations.push_back(mo);

   }


}

void MarkerTracker::sampleMarkerObservations(const IntrinsicMatrix& K)
{
    //clear output vector
    _markerObservationSamplingMap.clear();

    for(unsigned int i = 0; i < _markerObservations.size(); i++)
    {
        MarkerSampling ms;
        const MarkerObservation * mop = &_markerObservations[i];
        //read code for both local pose hypotheses and store it in the MarkerSampling

        //take a number of samples which is proportional to the ellipse' contour length
        int numSamples = 2*_markerObservations.at(i).getContourLength();

        ms.resizeCodeBuffer(numSamples*2);

        double radiusRing1 = 1./3.0-1.0/6.0; //this defines the middle of the first (inner) ring
        double radiusRing2 = 2./3.0-1.0/6.0; //this defines the middle of the second (middle) ring
        bool bres = true;
        //sample first ring
        for(int i=0; i<numSamples; i++)
        {
            bres &= getGreyValueAtProjectedPoint(radiusRing1*cos(i*2.0*CV_PI/numSamples), radiusRing1*sin(i*2.0*CV_PI/numSamples), mop->getP(1), K, ms.getCodeBufferRing1()[i]);
            bres &= getGreyValueAtProjectedPoint(radiusRing1*cos(i*2.0*CV_PI/numSamples), radiusRing1*sin(i*2.0*CV_PI/numSamples), mop->getP(2), K, ms.getCodeBufferRing2()[i]);

        }
        //sample second ring
        for(int i=0; i<numSamples; i++)
        {
            bres &= getGreyValueAtProjectedPoint(radiusRing2*cos(i*2.0*CV_PI/numSamples), radiusRing2*sin(i*2.0*CV_PI/numSamples), mop->getP(1), K, ms.getCodeBufferRing1()[i+numSamples]);
            bres &= getGreyValueAtProjectedPoint(radiusRing2*cos(i*2.0*CV_PI/numSamples), radiusRing2*sin(i*2.0*CV_PI/numSamples), mop->getP(2), K, ms.getCodeBufferRing2()[i+numSamples]);
        }

        if(bres) //output as observation only if sampling succeeded
        {
            (_markerObservationSamplingMap)[mop] = ms;
        }


    }


}

void MarkerTracker::computeCorrelations(const MarkerWorld& world)
{
    double angle = 0, correl = 0;
    int hypothesisNumber = 0;

    //clear previous entries
    _markerCorrelationTable.clear();

    //for each markerModel from the World
    for(MarkerWorld::const_MarkerPtrIterator itMarkerPtr = world.markerBegin();
        itMarkerPtr!= world.markerEnd();
        itMarkerPtr++)
    {
        //for each markerObservation
        for(MarkerObservationSamplingMap::const_iterator itObservation = _markerObservationSamplingMap.begin();
            itObservation!=_markerObservationSamplingMap.end();
            itObservation++)
        {
            //compute the best angular correlation, together with best angle and hypothesis number
            computeBestAngularCorrelation(**itMarkerPtr, itObservation->second,correl, angle, hypothesisNumber);
            //store it in the table
            (_markerCorrelationTable)[*itMarkerPtr][itObservation->first] = MarkerCorrelationItem(hypothesisNumber, angle, correl);

        }

    }

}

void MarkerTracker::computeAssignment()
{
    const double CORRELATION_THRESHOLD = 0.85;
    //This module assigns observations to markers models from the world
    //This early and very simple implementation simply assigns for each marker
    //the observation with best cross-correlation (if the correlation is high
    //enough though).
    //TODO: Replace with a better assignment technique like the Hungarian method

    //clear previous entries
    _markerCorrespondencyMap.clear();

    //for each marker from the model
    for(MarkerCorrelationTable::const_iterator itMarkerPtr = _markerCorrelationTable.begin();
        itMarkerPtr!=_markerCorrelationTable.end();
        itMarkerPtr++)
    {
        double bestCorrelation = -1.0;

        //for each markerObservation
        for(std::map<const MarkerObservation *, MarkerCorrelationItem>::const_iterator itObsPtr = itMarkerPtr->second.begin();
            itObsPtr!=itMarkerPtr->second.end();
            itObsPtr++)
        {
            if(itObsPtr->second._correlation>bestCorrelation && itObsPtr->second._correlation> CORRELATION_THRESHOLD)
            {
                bestCorrelation = itObsPtr->second._correlation;
                (_markerCorrespondencyMap)[itMarkerPtr->first] = MarkerRefinedObservation(itObsPtr->first, itObsPtr->second._angle, itObsPtr->second._hypothesis);
            }
        }


    }

}

void MarkerTracker::computeAllRigidBodyPoses(const MarkerWorld& world)
{
    _markerRigidBodyPoseMap.clear();
    
    //for each rigid body, compute single RigidBody pose
    for(MarkerWorld::const_iterator itRigidBody = world.begin();
        itRigidBody!= world.end();
        itRigidBody++)
    {
        PoseMatrix pose;
        bool res = computeSingleRigidBodyPose(&(*itRigidBody), pose);

        //if computation ok, put in the pose map
        if(res) _markerRigidBodyPoseMap[&(*itRigidBody)] = pose;
        
    }


}



bool MarkerTracker::computeSingleRigidBodyPose( const MarkerRigidBody* itRigidBody, PoseMatrix& pose )
{
    //outline of the algorithm:
    //1. get all planes present in the rigid body
    //    1.1 for each plane
    //        1.1.1 count number of visible markers on the plane
    //        1.1.2 compute homography for that plane according to the number of visible markers
    //    1.2 count number of visible planes ( visible plane == a plane that contains at least one visible marker)
    //    1.3 compute global pose of the rigid body from the homographies of each plane according to the number of visible planes
    
    //implementation of the algorithm:
    std::vector<std::pair<const MarkerPlane *, Homography> > visiblePlanes;
    visiblePlanes.clear();
    //1.1 for each plane
    for(MarkerRigidBody::const_MarkerPlaneIterator itPlane = itRigidBody->planeBegin();
        itPlane!= itRigidBody->planeEnd();
        itPlane++)
    {
        const MarkerPlane * planePtr = &(*itPlane); //current plane
        std::vector<MarkerCorrespondencyMap::const_iterator> visibleMarkers; //< visible markers for the current plane;
        visibleMarkers.clear();

        //1.1.1 count number of visible markers on that plane (and keep track of them)
        for(MarkerPlane::const_iterator itMarkerPtr= planePtr->begin();
            itMarkerPtr!= planePtr->end();
            itMarkerPtr++)
        {
            MarkerCorrespondencyMap::const_iterator itCorres = _markerCorrespondencyMap.find(*itMarkerPtr);
            if(itCorres != _markerCorrespondencyMap.end()) //if found
            {
                visibleMarkers.push_back(itCorres);
            }
        }

        //1.1.2 compute homography for that plane according to the number
        if(!visibleMarkers.empty())
        {
            Homography planarHomography = computePlanarHomography(*planePtr, visibleMarkers);
            visiblePlanes.push_back(make_pair(planePtr, planarHomography));
        }
    }

    // 1.2 count number of visible planes
    if(!visiblePlanes.empty())
    {
        pose = computePoseFromPlanes(visiblePlanes);
        return true;
    }
    else
    {
        return false;
    }

    

}

void MarkerTracker::computeChenHypotheses( const ConicMatrix& normalizedConic, PoseMatrix& P1, PoseMatrix& P2, bool forceZup, const IntrinsicMatrix& K )
{
    //decomposition matrices
    double matLData[9];
    CvMat matL = cvMat(3,3,CV_64FC1,matLData);
    double matEData[9];
    CvMat matE = cvMat(3,3,CV_64FC1,matEData);

    //Eigendecomposition of Cnorm, with eigenvalues sorting
    eigenDecomp(normalizedConic.cvMatPtr(), &matL, &matE);



    

    //Make E a positive rotation
    if(cvDet(&matE)<0) 
    {
        cvScale(&matE, &matE, -1.0);
    }


    //Chen's method for finding the 2 possible poses [R1, t1] and [R2, t2]
    //Notations from the paper:
    //"Camera Calibration with Two Arbitrary Coplanar Circles"
    //Qian Chen, Haiyuan Wu, and Toshikazu wada, ECCV 2004

    double L1 = cvmGet(&matL,0,0);
    double L2 = cvmGet(&matL,1,1);
    double L3 = cvmGet(&matL,2,2);

    double g = sqrt((L2-L3)/(L1-L3));
    double h = sqrt((L1-L2)/(L1-L3));

    double S1t, S2t, S3t; //temporary signs
    double S1[2], S2[2], S3[2];

    //test possible confs for signs S1, S2, S3
    int configurationCounter = 0;
    for(int conf = 0; conf<8; conf++)
    {
        S1t = (conf % 2)?-1.0:1.0;
        S2t = ((conf / 2)%2)?-1.0:1.0;
        S3t = (conf / 4)?-1.0:1.0;

        double z0 = S3t*L2/sqrt(-L1*L3);
        double c3 = z0 * (cvmGet(&matE, 2,0)*S2t*L3/L2*h - cvmGet(&matE, 2,2)*S1t*L1/L2*g);
        double n3 = (cvmGet(&matE, 2,0)*S2t*h - cvmGet(&matE, 2,2)*S1t*g);

        if(c3>0 && n3<0) // visibility test
        {
            if(configurationCounter>1)
            {
                std::cout << "VERY STRANGE: more than two possible solutions (" << configurationCounter+1 << ")" << std::endl;
            }
            else
            {
                S1[configurationCounter] = S1t;
                S2[configurationCounter] = S2t;
                S3[configurationCounter] = S3t;
            }
            configurationCounter++;

        }
    }// end for signs configuration loop

    //verify that exactly two solutions have been found
    if(configurationCounter!=2)
    {
        std::cout << "VERY STRANGE: less than two possible solutions (" << configurationCounter << ")" << std::endl;
    }

    double _R1data[9], _R2data[9], _T1data[3], _T2data[3];
    CvMat _R1 = cvMat(3,3,CV_64FC1,_R1data);
    CvMat _R2 = cvMat(3,3,CV_64FC1,_R2data);
    CvMat _T1 = cvMat(3,1,CV_64FC1,_T1data);
    CvMat _T2 = cvMat(3,1,CV_64FC1,_T2data);

    cvmSet(&_R1,0,0,g);
    cvmSet(&_R1,0,1,0);
    cvmSet(&_R1,0,2,S2[0]*h);
    cvmSet(&_R1,1,0,0);
    cvmSet(&_R1,1,1,-S1[0]);
    cvmSet(&_R1,1,2,0);
    cvmSet(&_R1,2,0,S1[0]*S2[0]*h);
    cvmSet(&_R1,2,1,0);
    cvmSet(&_R1,2,2,-S1[0]*g);
    cvmSet(&_T1,0,0,-S2[0]*sqrt((L1-L2)*(L2-L3))/L2);
    cvmSet(&_T1,1,0,0);
    cvmSet(&_T1,2,0,1.0);
    cvScale(&_T1,&_T1,S3[0]*L2/sqrt(-L1*L3));
    cvMatMul(&matE,&_R1, &_R1);
    cvMatMul(&_R1,&_T1, &_T1);

    cvmSet(&_R2,0,0,g);
    cvmSet(&_R2,0,1,0);
    cvmSet(&_R2,0,2,S2[1]*h);
    cvmSet(&_R2,1,0,0);
    cvmSet(&_R2,1,1,-S1[1]);
    cvmSet(&_R2,1,2,0);
    cvmSet(&_R2,2,0,S1[1]*S2[1]*h);
    cvmSet(&_R2,2,1,0);
    cvmSet(&_R2,2,2,-S1[1]*g);
    cvmSet(&_T2,0,0,-S2[1]*sqrt((L1-L2)*(L2-L3))/L2);
    cvmSet(&_T2,1,0,0);
    cvmSet(&_T2,2,0,1.0);
    cvScale(&_T2,&_T2,S3[1]*L2/sqrt(-L1*L3));
    cvMatMul(&matE,&_R2, &_R2);

    cvMatMul(&_R2,&_T2, &_T2);

        
    for(unsigned int i=0; i<3; i++)
    {
        cvmSet(P1.cvMatPtr(), i, 3, cvmGet(&_T1, i,0));
        cvmSet(P2.cvMatPtr(), i, 3, cvmGet(&_T2, i,0));
        for(unsigned int j=0; j<3; j++)
        {
            cvmSet(P1.cvMatPtr(), i, j, cvmGet(&_R1, i,j));
            cvmSet(P2.cvMatPtr(), i, j, cvmGet(&_R2, i,j));
        }
    }
        

    if(forceZup)
    {
        //compute for which hypothesis the projected Z is up in the image
        double orig_x1, orig_y1, up_x1, up_y1; 
        double orig_x2, orig_y2, up_x2, up_y2;

        project3DPointToPixel(K, P1, 0,0,0, orig_x1, orig_y1);
        project3DPointToPixel(K, P1, 0,0,1, up_x1, up_y1);
        project3DPointToPixel(K, P2, 0,0,0, orig_x2, orig_y2);
        project3DPointToPixel(K, P2, 0,0,1, up_x2, up_y2);
        
        

        double y1 = up_y1-orig_y1;
        double y2 = up_y2-orig_y2;

        if(y1>1.0 && y2<-1.0)
        {
            P1 = P2;

        }
        else if(y2>1.0 && y1<-1.0)
        {
            P2 = P1;
        }


    }
}

void MarkerTracker::eigenDecomp( CvMat* iC, CvMat* oDiag, CvMat* oEigenVec )
{
    //This function returns a decomposition so that
    // iC = oEigenVec * oDiag * oEigenVec^T

    //helper matrices
    double eigenvalues[3];
    CvMat EIGVALS = cvMat(3,1,CV_64F,eigenvalues);
    double eigenvectors[9];
    CvMat EIGVECS = cvMat(3,3,CV_64F,eigenvectors);

    //decompose with opencv built-in function
    CvMat * temp = cvCreateMat(3,3,CV_64F);
    cvCopy(iC, temp);
    cvEigenVV( temp, &EIGVECS, &EIGVALS, 0 );
    cvReleaseMat(&temp);


    if(cvmGet(&EIGVALS,0,0)<cvmGet(&EIGVALS,1,0) ||
        cvmGet(&EIGVALS,1,0)<cvmGet(&EIGVALS,2,0))
    {
        //cout <<"Bad sort order for eigenvalues." <<endl;
        sortEigenDecomp(&EIGVECS, &EIGVALS);

    }


    //construct output matrices
    cvZero(oDiag);
    cvmSet(oDiag,0,0,cvmGet(&EIGVALS,0,0));
    cvmSet(oDiag,1,1,cvmGet(&EIGVALS,1,0));
    cvmSet(oDiag,2,2,cvmGet(&EIGVALS,2,0));

    cvTranspose(&EIGVECS, oEigenVec);
}

void MarkerTracker::sortEigenDecomp( CvMat* eigenVecs, CvMat* eigenVals )
{
    double sortedEigenVecsData[9];
    CvMat sortedEigenVecs = cvMat(3,3,CV_64F,sortedEigenVecsData);
    double sortedEigenValsData[3];
    CvMat sortedEigenVals = cvMat(3,1,CV_64F,sortedEigenValsData);
    int L1, L2, L3;

    //look for L3
    if(cvmGet(eigenVals,0,0)<0) L3 = 0;
    else if(cvmGet(eigenVals,1,0)<0) L3 = 1;
    else if(cvmGet(eigenVals,2,0)<0) L3 = 2;

    //look for max
    double max=0;
    for(int i=0; i<3; i++)
    {
        if(i!=L3)
        {
            if(cvmGet(eigenVals,i,0)>max)
            {
                max = cvmGet(eigenVals,i,0);
                L1 = i;
            }
        }
    }
    for(int i=0; i<3; i++)
    {
        if(i!=L3 && i!=L1)
        {
            L2 = i;
        }
    }

    //now it is sorted. Exchange rows
    cvmSet(&sortedEigenVals,0,0,cvmGet(eigenVals,L1,0));
    cvmSet(&sortedEigenVals,1,0,cvmGet(eigenVals,L2,0));
    cvmSet(&sortedEigenVals,2,0,cvmGet(eigenVals,L3,0));

    for(int i=0; i<3; i++)
    {
        cvmSet(&sortedEigenVecs,0,i,cvmGet(eigenVecs,L1,i));
        cvmSet(&sortedEigenVecs,1,i,cvmGet(eigenVecs,L2,i));
        cvmSet(&sortedEigenVecs,2,i,cvmGet(eigenVecs,L3,i));
    }

    cvScale(&sortedEigenVals,eigenVals);
    cvScale(&sortedEigenVecs,eigenVecs);
}

void MarkerTracker::showMarkerObservations(const IntrinsicMatrix& K)
{
    for(unsigned int i = 0; i < _markerObservations.size(); i++)
    {

        //draw 2 patches and up vectors (for debugging purposes)
        PoseMatrix p1 = _markerObservations.at(i).getP(1);
        PoseMatrix p2 = _markerObservations.at(i).getP(2);
        drawPatch(getDebugImage(), p1, K, CV_RGB(0,0,255));
        drawPatch(getDebugImage(), p2, K, CV_RGB(0,255,255));

    }

}

void MarkerTracker::showPose(const IntrinsicMatrix& K, const MarkerWorld& world)
{
    int i=0;
    for(MarkerWorld::const_iterator itRigidBody = world.begin();
        itRigidBody!= world.end();
        itRigidBody++, i++)
    {
        MarkerRigidBodyPoseMap::iterator itPose = _markerRigidBodyPoseMap.find(&(*itRigidBody));
        if(itPose!=_markerRigidBodyPoseMap.end())
        {
            PoseMatrix P = itPose->second;

            double orig_x, orig_y, Z_x, Z_y, X_x, X_y, Y_x, Y_y; 
            project3DPointToPixel(K, P, 0,0,0, orig_x, orig_y);
            project3DPointToPixel(K, P, 0,0,0.05, Z_x, Z_y);
			project3DPointToPixel(K, P, 0.05, 0, 0, X_x, X_y);
			project3DPointToPixel(K, P, 0, 0.05, 0, Y_x, Y_y);
            

            cvLine(getDebugImage(), cvPointFrom32f(cvPoint2D32f(orig_x, orig_y)), cvPointFrom32f(cvPoint2D32f(X_x, X_y)), CV_RGB(255,0,0),3);
            cvLine(getDebugImage(), cvPointFrom32f(cvPoint2D32f(orig_x, orig_y)), cvPointFrom32f(cvPoint2D32f(Y_x, Y_y)), CV_RGB(0,255,0),3);
            cvLine(getDebugImage(), cvPointFrom32f(cvPoint2D32f(orig_x, orig_y)), cvPointFrom32f(cvPoint2D32f(Z_x, Z_y)), CV_RGB(0,0,255),3);
            

        }
        

    }


}

void MarkerTracker::showMarkerAssignments(const IntrinsicMatrix& K, const MarkerWorld& world)
{
    //show found markers
    CvFont greenFont, blackFont;
    cvInitFont( &blackFont, 1, 1.0, 1.0, 0,2,16);
    cvInitFont( &greenFont, 1, 1.0, 1.0, 0,1,16);

    for(MarkerWorld::const_MarkerPtrIterator itMarkerPtr = world.markerBegin();
        itMarkerPtr != world.markerEnd();
        itMarkerPtr++)
    {
        MarkerCorrespondencyMap::const_iterator itCorres = _markerCorrespondencyMap.find(*itMarkerPtr);
        if(itCorres != _markerCorrespondencyMap.end())
        {

            int hypothesis = itCorres->second._hypothesis;
            if(hypothesis==0)
            {
                continue;
            }

            //show MarkerObservation
            //draw 2 patches and up vectors (for debugging purposes)
            PoseMatrix p1 = itCorres->second._markerObservationPtr->getP(itCorres->second._hypothesis);
            drawPatch(getDebugImage(), p1, K, CV_RGB(0,0,255));

            double pixel_x, pixel_y;
            bool bres;
            bres = project3DPointToPixel(K, itCorres->second._markerObservationPtr->getP(hypothesis),0,0,0,pixel_x, pixel_y);

            if(bres)
            {
                cvPutText(getDebugImage(), (*itMarkerPtr)->getName().c_str(), cvPoint((int)(pixel_x+0.5), (int)(pixel_y+0.5)),&blackFont, CV_RGB(0,0,0));
                cvPutText(getDebugImage(), (*itMarkerPtr)->getName().c_str(), cvPoint((int)(pixel_x+0.5), (int)(pixel_y+0.5)),&greenFont, CV_RGB(0,255,0));
            }

        }
    }


}

void MarkerTracker::drawPatch( IplImage* im, const PoseMatrix& P, const IntrinsicMatrix& K, CvScalar color )
{
    double orig_x, orig_y, up_x, up_y, c1_x, c1_y, c2_x, c2_y, c3_x, c3_y, c4_x, c4_y; 

    project3DPointToPixel(K, P, 0,0,0, orig_x, orig_y);
    project3DPointToPixel(K, P, 0,0,1, up_x, up_y);
    project3DPointToPixel(K, P, 1,1,0, c1_x, c1_y);
    project3DPointToPixel(K, P, 1,-1,0, c2_x, c2_y);
    project3DPointToPixel(K, P, -1,-1,0, c3_x, c3_y);
    project3DPointToPixel(K, P, -1,1,0, c4_x, c4_y);
    
    //cvLine(im, cvPointFrom32f(cvPoint2D32f(orig_x, orig_y)), cvPointFrom32f(cvPoint2D32f(up_x, up_y)), color,3);
    //cvLine(im, cvPointFrom32f(cvPoint2D32f(c1_x, c1_y)), cvPointFrom32f(cvPoint2D32f(c2_x, c2_y)), color,3);
    //cvLine(im, cvPointFrom32f(cvPoint2D32f(c2_x, c2_y)), cvPointFrom32f(cvPoint2D32f(c3_x, c3_y)), color,3);
    //cvLine(im, cvPointFrom32f(cvPoint2D32f(c3_x, c3_y)), cvPointFrom32f(cvPoint2D32f(c4_x, c4_y)), color,3);
    //cvLine(im, cvPointFrom32f(cvPoint2D32f(c4_x, c4_y)), cvPointFrom32f(cvPoint2D32f(c1_x, c1_y)), color,3);
    

    for(double a=0; a<2*3.1415; a+=3.1415/60.0)
    {
        double b=a+3.1415/60;
        double ax, ay, bx, by;
        project3DPointToPixel(K, P, cos(a),sin(a),0, ax, ay);
        project3DPointToPixel(K, P, cos(b),sin(b),0, bx, by);
        cvLine(im, cvPointFrom32f(cvPoint2D32f(ax, ay)), cvPointFrom32f(cvPoint2D32f(bx, by)), CV_RGB(255,255,0),3);

    }
}

bool MarkerTracker::getGreyValueAtProjectedPoint( double x, double y, const PoseMatrix& P, const IntrinsicMatrix& K, double& val )
{
    //compute projection
    double pt2d_x, pt2d_y;

    bool bres= project3DPointToPixel(K, P, x, y, 0, pt2d_x, pt2d_y);

    if(!bres)
    {
        return false;
    }

    int xi=(int)(pt2d_x);
    int yi=(int)(pt2d_y);
    double xd = pt2d_x;
    double yd = pt2d_y;

    uchar valmm=0, valMM=0, valmM=0, valMm=0;

    if(xi>0 && xi<_grey->width-1 && yi>0 && yi<_grey->height-1)
    {

        valmm = ((uchar *)(_grey->imageData + (yi)*_grey->widthStep))[(xi)];
        valMM = ((uchar *)(_grey->imageData + (yi+1)*_grey->widthStep))[(xi+1)];
        valmM = ((uchar *)(_grey->imageData + (yi+1)*_grey->widthStep))[(xi)];
        valMm = ((uchar *)(_grey->imageData + (yi)*_grey->widthStep))[(xi+1)];


        //bilinear interpolation

        val = valMM*(xd-xi)*(yd-yi)+valmm*(xi+1-xd)*(yi+1-yd)+valMm*(xd-xi)*(yi+1-yd)+valmM*(xi+1-xd)*(yd-yi);
        return true;
    }
    else
    {
        return false;
    }

    return true;
}

bool MarkerTracker::project3DPointToPixel( const IntrinsicMatrix& K, const PoseMatrix& P, double X, double Y, double Z, double& x, double& y ) const
{
    double pt3d_world_data[4]={X,Y,Z,1};
    CvMat pt3d_world = cvMat(4,1,CV_64FC1, pt3d_world_data);
    double pt3d_camera_data[3];
    CvMat pt3d_camera = cvMat(3,1,CV_64FC1, pt3d_camera_data);
    
    cvMatMul(P.cvMatPtr(), &pt3d_world, &pt3d_camera);
    
    double pt2d_homog_data[3];
    CvMat pt2d_homog = cvMat(3,1,CV_64FC1, pt2d_homog_data);
    
    cvMatMul(K.cvMatPtr(), &pt3d_camera, &pt2d_homog);

    
    if(pt2d_homog_data[2]<=DBL_EPSILON) return false;

    x = pt2d_homog_data[0]/pt2d_homog_data[2];
    y = pt2d_homog_data[1]/pt2d_homog_data[2];

    
    return true;
    

}

void MarkerTracker::computeBestAngularCorrelation( const MarkerModel& model, const MarkerSampling& sampling, double& correl, double &angle, int& hypothesis )
{
    angle = correl = 0.0;
    hypothesis = 0;

    //This is an implementation of the pyramidal cross-correlation (from AP)
    //It computes a cross correlation of the observation samples with the model
    //at increasing resolutions to efficiently find the right angle without 
    //computing too much correlation.
    //This method produces jitter if the sampling is not high enough
    
    double maxCorrel1 = -10000.0; 
    double maxCorrel2 = -10000.0; 
    int offsetNumber1 = 0;
    int offsetNumber2 = 0;

    int curResolution = 12; //number of bins in the outer ring
    int curMin = 0;
    int curMax = curResolution;
    int offset;
    double correl1, correl2;

    //find approximate orientation
    int numSamples = sampling.getNumSamples();

    while(curResolution<=numSamples)
    {
        maxCorrel1 = -10000.0;
        for(int offsetCounter=curMin; offsetCounter<curMax; offsetCounter++)
        {
            //compute cross correlation

            //modulo correction
            offset = offsetCounter;
            if(offset<0) offset+=curResolution;
            if(offset>=curResolution) offset-=curResolution;



            correl1 = crossCorrelation(offset, curResolution, model.getModel(), sampling.getCodeBufferRing1(), numSamples);



            //look for best crosscorrelation
            if(correl1>maxCorrel1)
            {
                maxCorrel1 = correl1;
                offsetNumber1 = offset;
            }


        }// end for offset loop

        //now offset for current resolution is found, go to next step resolution
        curResolution *= 2;
        curMin = 2*offsetNumber1-2;
        curMax = 2*offsetNumber1+2;



        //curMin = 0;
        //curMax = curResolution;


    }

    curResolution = 12; //num of bins in outer ring
    curMin = 0;
    curMax = curResolution;

    //find approximate orientation

    while(curResolution<=numSamples)
    {
        maxCorrel2 = -10000.0;
        for(int offsetCounter=curMin; offsetCounter<curMax; offsetCounter++)
        {
            //compute cross correlation

            //modulo correction
            offset = offsetCounter;
            if(offset<0) offset+=curResolution;
            if(offset>=curResolution) offset-=curResolution;



            correl2 = crossCorrelation(offset, curResolution, model.getModel(), sampling.getCodeBufferRing2(), numSamples);



            //look for best crosscorrelation
            if(correl2>maxCorrel2)
            {
                maxCorrel2 = correl2;
                offsetNumber2 = offset;
            }


        }// end for offset loop

        //now offset for current resolution is found, go to next step resolution
        curResolution *= 2;
        curMin = 2*offsetNumber2-2;
        curMax = 2*offsetNumber2+2;



        //curMin = 0;
        //curMax = curResolution;


    }

    curResolution /=2;



    if(maxCorrel1>0.8 && maxCorrel2>0.8)
    {
        //std::cout << maxCorrel1 << ", " << maxCorrel2 << " - ";
        //if(maxCorrel1>maxCorrel2) cout << maxCorrel2/maxCorrel1 << endl;
        //else cout << maxCorrel1/maxCorrel2 << endl;
    }
    if(maxCorrel1>maxCorrel2)
    {
        //this test is needed to discard cases where no distinctions can be made between the two chen hypotheses
        //TODO: change it, in particular check if the two solutions agree at least on the model (after complete assignment)
        //because hypothesis is not important when at least two markers have been detected on a plane
        //if(maxCorrel2/maxCorrel1<0.98)
        {
            angle = offsetNumber1*2.0*CV_PI/curResolution;
            hypothesis = 1;
            correl = maxCorrel1;
            return;
        }
        //else return;
    }
    else
    {
        //this test is needed to discard cases where no distinctions can be made between the two chen hypotheses
        //TODO: change it, in particular check if the two solutions agree at least on the model (after complete assignment)
        //because hypothesis is not important when at least two markers have been detected on a plane
        //if(maxCorrel1/maxCorrel2<0.98)
        {
            angle = offsetNumber2*2.0*CV_PI/curResolution;
            hypothesis = 2;
            correl = maxCorrel2;
            return;
        }
        //else return;
    }


    
}

double MarkerTracker::crossCorrelation( int offset, int resolution, const int * model, const double * samplesVec, int size )
{
    double sum1=0, sum2=0, sumsquares1=0, sumsquares2=0, sumcorr=0;
    for(int index=0; index<size; index++)
    {
        double val = ((index < size / 2)?(model[(8*((index+(offset*size / 2)/resolution)%(size / 2)))/size]):(model[(24*(((index-size / 2)+(offset*size / 2)/resolution)%(size / 2)))/size+4]));//getModelValue(i, offset, resolution, size);
        sum1+=val;
        sum2+=samplesVec[index];
        sumsquares1+=val*val;
        sumsquares2+=samplesVec[index]*samplesVec[index];
    }

    double mean1 = sum1/size;
    double mean2 = sum2/size;

    double sigma1 = sqrt(sumsquares1/size-mean1*mean1);
    double sigma2 = sqrt(sumsquares2/size-mean2*mean2);

    double epsilon = 1.0e-08;
    if (sigma1<epsilon)
    {
        if (sigma2<epsilon)
        {
            return -3.0;
        }
        else
        {
            return -1.0;
        }
    }
    if(sigma2<epsilon)
    {
        return -2.0;
    }

    for(int index=0; index<size; index++)
    {
        sumcorr+=(((index < size / 2)?(model[(8*((index+(offset*size / 2)/resolution)%(size / 2)))/size]):(model[(24*(((index-size / 2)+(offset*size / 2)/resolution)%(size / 2)))/size+4]))-mean1)*(samplesVec[index]-mean2);
    }

    return sumcorr/(sigma1*sigma2*(size-1.0));
}

Homography MarkerTracker::computePlanarHomography( const MarkerPlane& plane,  const std::vector<MarkerCorrespondencyMap::const_iterator>& visibleMarkers) const
{
    if(visibleMarkers.size()==1)
    {
        return computePlanarHomography1(plane, visibleMarkers);
    }
    else 
    {
        //not implemented yet
        //std::cout << "MarkerTracker::computePlanarHomography: found " << visibleMarkers.size() << "markers in one plane, but current version supports only single markers" << std::endl;
        return computePlanarHomography1(plane, visibleMarkers);
        
    }
    
    return Homography();
}



Homography MarkerTracker::computePlanarHomography1( const MarkerPlane& plane, const std::vector<MarkerCorrespondencyMap::const_iterator>& visibleMarkers ) const
{
    assert(visibleMarkers.size()==1);
    //simply use the last steps of already precomputed Chen's method

    const MarkerModel * markerModelPtr = visibleMarkers[0]->first;
    const MarkerObservation * markerObsPtr = visibleMarkers[0]->second._markerObservationPtr;
    double angle = visibleMarkers[0]->second._angle;
    PoseMatrix P = markerObsPtr->getP(visibleMarkers[0]->second._hypothesis);

    MatrixCv3x3 localR;
    VectorCv3 localT;
    for(unsigned int i=0; i<3; i++)
    {
        for(unsigned int j=0; j<3; j++)
        {
            cvmSet(localR.cvMatPtr(),i,j, cvmGet(P.cvMatPtr(), i,j));
        }
         cvmSet(localT.cvMatPtr(),i,0, cvmGet(P.cvMatPtr(), i,3));
    }


    // compute last orientation around Z-axis
    MatrixCv3x3 Rz;

    cvmSet(Rz.cvMatPtr(), 0, 0, cos(angle));
    cvmSet(Rz.cvMatPtr(), 0, 1, sin(angle));
    cvmSet(Rz.cvMatPtr(), 0, 2, 0);
    cvmSet(Rz.cvMatPtr(), 1, 0, -sin(angle));
    cvmSet(Rz.cvMatPtr(), 1, 1, cos(angle));
    cvmSet(Rz.cvMatPtr(), 1, 2, 0);
    cvmSet(Rz.cvMatPtr(), 2, 0, 0);
    cvmSet(Rz.cvMatPtr(), 2, 1, 0);
    cvmSet(Rz.cvMatPtr(), 2, 2, 1);

    MatrixCv3x3 globalR;
    cvMatMul(localR.cvMatPtr(), Rz.cvMatPtr(), globalR.cvMatPtr());

    //now, use the definition of the plane to compute
    //the transformation in the plane's coordinate frame

    //definition of Twm (plane dependent)
    VectorCv3 Twm;
    cvScale(plane.getNormal().cvMatPtr(), Twm.cvMatPtr(), plane.getDist());
    cvSub(Twm.cvMatPtr(),markerModelPtr->getCenter().cvMatPtr(), Twm.cvMatPtr() );
    
    //definition of Rwm (plane dependent)
    MatrixCv3x3 Rwm;
    for(unsigned int i=0; i<3; i++)
    {
        cvmSet(Rwm.cvMatPtr(), 0, i, cvmGet(plane.getU().cvMatPtr(), i, 0));
        cvmSet(Rwm.cvMatPtr(), 1, i, cvmGet(plane.getV().cvMatPtr(), i, 0));
        cvmSet(Rwm.cvMatPtr(), 2, i, cvmGet(plane.getNormal().cvMatPtr(), i, 0));
    }
    
    //computation of tg = R*twm + vecT
    VectorCv3 globalT;
    VectorCv3 Twm2;
    VectorCv3 Twm3;
    VectorCv3 midT;
    cvMatMul(Rwm.cvMatPtr(), Twm.cvMatPtr(), Twm2.cvMatPtr());
    cvMatMul(globalR.cvMatPtr(), Twm2.cvMatPtr(), Twm3.cvMatPtr()); 
    cvScale (localT.cvMatPtr(), midT.cvMatPtr(), markerModelPtr->getRadius());
    cvAdd(midT.cvMatPtr(), Twm3.cvMatPtr(), globalT.cvMatPtr() );

    //set the homography
    Homography H;
    for(int i=0; i<3;i++)
    {
        cvmSet(H.cvMatPtr(),i,0,cvmGet(globalR.cvMatPtr(),i,0));
        cvmSet(H.cvMatPtr(),i,1,cvmGet(globalR.cvMatPtr(),i,1));
        cvmSet(H.cvMatPtr(),i,2,cvmGet(globalT.cvMatPtr(),i,0));
    }

    return H;
}

Homography MarkerTracker::computePlanarHomography2( const MarkerPlane& plane, const std::vector<MarkerCorrespondencyMap::const_iterator>& visibleMarkers ) const
{
    //not implemented yet
    Homography H;
    return H;
}

Homography MarkerTracker::computePlanarHomography3orMore( const MarkerPlane& plane, const std::vector<MarkerCorrespondencyMap::const_iterator>& visibleMarkers ) const
{
    //not implemented yet
    Homography H;
    return H;
}

PoseMatrix MarkerTracker::computePoseFromPlanes( const std::vector<std::pair<const MarkerPlane*, Homography> >& visiblePlanes ) const
{
    //single plane pose computation
    if(visiblePlanes.size()==1)
    {
        return computePoseFromPlanes1( visiblePlanes );
    }
    //multiple plane pose computation
    else
    {
        //not implemented yet
        //std::cout << "MarkerTracker::computePoseFromPlanes: found " << visiblePlanes.size() << "planes in one rigid body, but current version supports only single plane" << std::endl;
        return computePoseFromPlanes1( visiblePlanes );
    }
}

PoseMatrix MarkerTracker::computePoseFromPlanes1( const std::vector<std::pair<const MarkerPlane*, Homography> >& visiblePlanes ) const
{
    PoseMatrix P;
    //single plane pose computation
    assert(visiblePlanes.size()==1);
    
    //1. compute plane pose from homography

    MatrixCv3x3 R;
    VectorCv3 t;
    planeHomographyToPose(visiblePlanes[0].second, R, t);

    //2. compute world pose from plane pose

    //definition of Rwm (plane dependent)
    MatrixCv3x3 Rwm;
    for(unsigned int i=0; i<3; i++)
    {
        cvmSet(Rwm.cvMatPtr(), 0, i, cvmGet(visiblePlanes[0].first->getU().cvMatPtr(), i, 0));
        cvmSet(Rwm.cvMatPtr(), 1, i, cvmGet(visiblePlanes[0].first->getV().cvMatPtr(), i, 0));
        cvmSet(Rwm.cvMatPtr(), 2, i, cvmGet(visiblePlanes[0].first->getNormal().cvMatPtr(), i, 0));
    }


    //definition of Twm (plane dependent)
    VectorCv3 Twm;
    cvmSet(Twm.cvMatPtr(), 0, 0, 0);
    cvmSet(Twm.cvMatPtr(), 1, 0, 0);
    cvmSet(Twm.cvMatPtr(), 2, 0, - visiblePlanes[0].first->getDist());


    VectorCv3 t1;
    cvMatMulAdd(R.cvMatPtr(), Twm.cvMatPtr(), t.cvMatPtr(), t1.cvMatPtr());
    MatrixCv3x3 R1;
    cvMatMul(R.cvMatPtr(), Rwm.cvMatPtr(), R1.cvMatPtr());

    for(int i=0; i<3; i++)
    {
        for(int j=0; j<3; j++)
        {
            cvmSet(P.cvMatPtr(),i,j,cvmGet(R1.cvMatPtr(),i,j));

        }
        cvmSet(P.cvMatPtr(),i,3,cvmGet(t1.cvMatPtr(),i,0));
    }

    return P;
    
}

void MarkerTracker::planeHomographyToPose( const Homography& H, MatrixCv3x3& R, VectorCv3& t ) const
{
    int i;
    
    // Vectors holding columns of _homogMat and R:
    double a_H1[3];
    CvMat  m_H1 = cvMat( 3, 1, CV_64FC1, a_H1 );
    for( i = 0; i < 3; i++ ) cvmSet( &m_H1, i, 0, cvmGet( H.cvMatPtr(), i, 0 ) );

    double a_H2[3];
    CvMat  m_H2 = cvMat( 3, 1, CV_64FC1, a_H2 );
    for( i = 0; i < 3; i++ ) cvmSet( &m_H2, i, 0, cvmGet( H.cvMatPtr(), i, 1 ) );

    double a_H3[3];
    CvMat  m_H3 = cvMat( 3, 1, CV_64FC1, a_H3 );
    for( i = 0; i < 3; i++ ) cvmSet( &m_H3, i, 0, cvmGet( H.cvMatPtr(), i, 2 ) );


    double a_R1[3];
    CvMat  m_R1 = cvMat( 3, 1, CV_64FC1, a_R1 );

    double a_R2[3];
    CvMat  m_R2 = cvMat( 3, 1, CV_64FC1, a_R2 );

    double a_R3[3];
    CvMat  m_R3 = cvMat( 3, 1, CV_64FC1, a_R3 );





    // Search next orthonormal matrix:
    if( cvNorm( &m_H1, NULL, CV_L2, NULL ) != 0 )
    {
        double lambda = 1.00/cvNorm( &m_H1, NULL, CV_L2, NULL );

        // Calculate Translation Vector T (- because of its definition):
        cvScale( &m_H3, t.cvMatPtr(), lambda);

        if(cvmGet(t.cvMatPtr(), 2,0)<0) 
        {
            lambda = - lambda;
            cvScale(t.cvMatPtr(), t.cvMatPtr(), -1.0);
        }
        
        // Create normalized R1 & R2:
        cvScale( &m_H1, &m_R1, lambda);
        cvScale( &m_H2, &m_R2, lambda);

        // Get R3 orthonormal to R1 and R2:
        cvCrossProduct( &m_R1, &m_R2, &m_R3 );

        // Put the rotation column vectors in the rotation matrix:
        for( i = 0; i < 3; i++ ){
            cvmSet( R.cvMatPtr(), i, 0,  cvmGet( &m_R1, i, 0 ) );
            cvmSet( R.cvMatPtr(), i, 1,  cvmGet( &m_R2, i, 0 ) );
            cvmSet( R.cvMatPtr(), i, 2,  cvmGet( &m_R3, i, 0 ) );
        }



        // Transformation of R into - in Frobenius sense - next orthonormal matrix:
        double a_W[9];	CvMat  m_W  = cvMat( 3, 3, CV_64FC1, a_W  );
        double a_U[9];	CvMat  m_U  = cvMat( 3, 3, CV_64FC1, a_U  );
        double a_Vt[9];	CvMat  m_Vt = cvMat( 3, 3, CV_64FC1, a_Vt );
        cvSVD( R.cvMatPtr(), &m_W, &m_U, &m_Vt, CV_SVD_MODIFY_A | CV_SVD_V_T );
        cvMatMul( &m_U, &m_Vt, R.cvMatPtr() );

        

        return;
    }
}

void MarkerTracker::writeData( const MarkerWorld& world, bool* markerVisible, double* modelviewMatrix )
{
    int counter=0;
    for(MarkerWorld::const_iterator itRigidBody = world.begin();
        itRigidBody!= world.end();
        itRigidBody++)
    {
        MarkerRigidBodyPoseMap::iterator itPose = _markerRigidBodyPoseMap.find(&(*itRigidBody));
        if(itPose!=_markerRigidBodyPoseMap.end())
        {
            if(markerVisible!=0) markerVisible[counter]=true;
            if(modelviewMatrix!=0)
            {
                //first column of rotation
                modelviewMatrix[0+16*counter] = cvmGet(itPose->second.cvMatPtr(), 0, 0);
                modelviewMatrix[1+16*counter] = cvmGet(itPose->second.cvMatPtr(), 1, 0);
                modelviewMatrix[2+16*counter] = cvmGet(itPose->second.cvMatPtr(), 2, 0);
                modelviewMatrix[3+16*counter] = 0.0;

                //second column of rotation
                modelviewMatrix[4+16*counter] = cvmGet(itPose->second.cvMatPtr(), 0, 1);
                modelviewMatrix[5+16*counter] = cvmGet(itPose->second.cvMatPtr(), 1, 1);
                modelviewMatrix[6+16*counter] = cvmGet(itPose->second.cvMatPtr(), 2, 1);
                modelviewMatrix[7+16*counter] = 0.0;

                //third column of rotation
                modelviewMatrix[8+16*counter] = cvmGet(itPose->second.cvMatPtr(), 0, 2);
                modelviewMatrix[9+16*counter] = cvmGet(itPose->second.cvMatPtr(), 1, 2);
                modelviewMatrix[10+16*counter] = cvmGet(itPose->second.cvMatPtr(), 2, 2);
                modelviewMatrix[11+16*counter] = 0.0;

                //translation
                modelviewMatrix[12+16*counter] = cvmGet(itPose->second.cvMatPtr(), 0, 3);
                modelviewMatrix[13+16*counter] = cvmGet(itPose->second.cvMatPtr(), 1, 3);
                modelviewMatrix[14+16*counter] = cvmGet(itPose->second.cvMatPtr(), 2, 3);
                modelviewMatrix[15+16*counter] = 1.0;
            }
            
            
        }
        else
        {
            markerVisible[counter]=false;
        }
        counter++;

       
        
    }

}