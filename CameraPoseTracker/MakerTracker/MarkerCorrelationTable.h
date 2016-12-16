#pragma once
#include "MarkerModel.h"
#include "MarkerObservation.h"
#include <map>

class MarkerCorrelationItem
{   
public:
    MarkerCorrelationItem():_hypothesis(0), _angle(0.0), _correlation(0.0){}
    MarkerCorrelationItem(int h, double a, double c):_hypothesis(h), _angle(a), _correlation(c){}
    int    _hypothesis;
    double _angle;
    double _correlation;
};

class MarkerCorrelationTable :
    public std::map<const MarkerModel *, std::map<const MarkerObservation *, MarkerCorrelationItem> >
{
public:
    MarkerCorrelationTable(void);
    ~MarkerCorrelationTable(void);
};
