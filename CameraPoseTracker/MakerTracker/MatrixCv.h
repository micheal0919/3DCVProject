#pragma once
#include "opencv\cv.h"
template<int Rows, int Cols>
class MatrixCv
{
public:
    MatrixCv()
    {
        _cvMat = cvCreateMat(Rows,Cols,CV_64FC1);
    }

    MatrixCv(const MatrixCv<Rows, Cols>& source)
    {
        _cvMat = cvCreateMat(Rows,Cols,CV_64FC1);
        *this = source;
    }
    MatrixCv<Rows, Cols>& operator=(const MatrixCv<Rows, Cols>& source)
    {
        cvScale(source.cvMatPtr(), this->cvMatPtr());
        return *this;
    }
    ~MatrixCv()
    {
        cvReleaseMat(&_cvMat);
    }
    CvMat* cvMatPtr() const
    {
        return _cvMat;
    }


private:
    CvMat * _cvMat;
};

typedef MatrixCv<3,3> MatrixCv3x3;
typedef MatrixCv<3,3> ConicMatrix;
typedef MatrixCv<3,3> IntrinsicMatrix;
typedef MatrixCv<3,4> PoseMatrix;
typedef MatrixCv<3,3> Homography;
typedef MatrixCv<3,1> VectorCv3;
typedef MatrixCv<5,1> VectorCv5;