#include "MarkerObservation.h"



void MarkerObservation::setNormalizedConic( const ConicMatrix& conicMatrix )
{
    _normalizedConic = conicMatrix;
}

void MarkerObservation::setLocalPoseHypotheses( const PoseMatrix& P1, const PoseMatrix& P2 )
{
   _P1 = P1;
   _P2 = P2;
}



