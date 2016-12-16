#include "Ellipse2D.h"

Ellipse2D::Ellipse2D():
_numPoints(0)
{
}

Ellipse2D::Ellipse2D( const ConicMatrix& conic ):
_numPoints(0)
{
    _conic = conic;
}




void Ellipse2D::convertConicToDrawingParameters( CvPoint& center, CvSize& size, double& angle ) const
{
    double a, b, c, d, e, f;
    a = cvmGet(_conic.cvMatPtr(), 0, 0);
    b = cvmGet(_conic.cvMatPtr(), 0, 1);
    c = cvmGet(_conic.cvMatPtr(), 1, 1);
    d = cvmGet(_conic.cvMatPtr(), 0, 2);
    e = cvmGet(_conic.cvMatPtr(), 1, 2);
    f = cvmGet(_conic.cvMatPtr(), 2, 2);

    double x0, y0, w, h;

    // extract ellipse center, size and angle from above values


    //1) find center of ellipse
    //it satisfy equation
    //| a    b | *  | x0 | +  | d | = |0 |
    //| b    c |    | y0 |    | e |   |0 |

    double idet = a * c - b * b;
    idet = idet > DBL_EPSILON ? 1./idet : 0;

    // we must normalize (a b c d e f ) to fit (4ac-4b^2=1)
    double scale = sqrt( 0.25 * idet );

    if( scale < DBL_EPSILON )
    {
        center = cvPoint(0,0);
        size = cvSize(0,0);
        angle = 0;
        return;
    }

    a *= scale;
    b *= scale;
    c *= scale;
    d *= scale;
    e *= scale;
    f *= scale;


    x0 = (-d * c + e * b) * 4.;
    y0 = (-a * e + d * b) * 4.;

    // offset ellipse to (x0,y0)
    // new f == F(x0,y0)
    f += a * x0 * x0 + b * 2. * x0 * y0 + c * y0 * y0 + d * 2. * x0 + e * 2. * y0;

    if( fabs(f) < DBL_EPSILON )
    {
        center = cvPoint(0,0);
        size = cvSize(0,0);
        angle = 0;

        return;
    }

    scale = -1. / f;
    // normalize to f = 1
    a *= scale;
    b *= scale;
    c *= scale;

    // extract axis of ellipse
    // one more eigenvalue operation
    double S[4], eigenvectors[4], eigenvalues[2];
    S[0] = a;
    S[1] = S[2] = b;
    S[3] = c;

    CvMat _S = cvMat( 2, 2, CV_64F, S );
    CvMat _EIGVECS = cvMat( 2, 2, CV_64F, eigenvectors );
    CvMat _EIGVALS = cvMat( 1, 2, CV_64F, eigenvalues );
    cvSVD( &_S, &_EIGVALS, &_EIGVECS, 0, CV_SVD_MODIFY_A + CV_SVD_U_T );

    // extract axis length from eigenvectors
    w = (float)(2./sqrt(eigenvalues[0]));
    h = (float)(2./sqrt(eigenvalues[1]));

    // calc angle
    angle = (float)(180 - atan2(eigenvectors[2], eigenvectors[3])*180/CV_PI);

    int xi = (int)(x0+0.5);
    int yi = (int)(y0+0.5);
    int hwi = (int)(w/2.0 + 0.5);
    int hhi = (int)(h/2.0 + 0.5);

    center =  cvPoint(xi, yi);
    size = cvSize(hwi, hhi);
    angle *= -1.0;

}

void Ellipse2D::setNumPoints( int numPoints )
{
    _numPoints = numPoints;
}

void Ellipse2D::setConic( const ConicMatrix& conic )
{
    _conic = conic;
}

int Ellipse2D::getNumPoints() const
{
    return _numPoints;
}

ConicMatrix Ellipse2D::getConic() const
{
    return _conic;
}