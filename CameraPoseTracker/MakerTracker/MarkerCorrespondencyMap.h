#pragma once
#include "MarkerModel.h"
#include "MarkerObservation.h"
#include <map>


class MarkerRefinedObservation
{
public:
    MarkerRefinedObservation(): _markerObservationPtr(0), _angle(0.0), _hypothesis(0){}
    MarkerRefinedObservation(const MarkerObservation * mp, double a, int h): _markerObservationPtr(mp), _angle(a), _hypothesis(h){}
    const MarkerObservation *     _markerObservationPtr;
    double                  _angle;
    int                     _hypothesis;
};

class MarkerCorrespondencyMap :
    public std::map<const MarkerModel *, MarkerRefinedObservation>
{
public:
    MarkerCorrespondencyMap(void);
    ~MarkerCorrespondencyMap(void);

    //Factory support
    static std::string getTypeIdStatic(void) {return "MarkerCorrespondencyMap";}
    virtual std::string getTypeId(void) const { return getTypeIdStatic(); }

};
