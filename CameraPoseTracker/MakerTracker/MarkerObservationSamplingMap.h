#pragma once
#include "MarkerObservation.h"
#include "MarkerSampling.h"
#include <map>

class MarkerObservationSamplingMap :
    public std::map<const MarkerObservation *, MarkerSampling>
{
public:
    MarkerObservationSamplingMap(void);
    ~MarkerObservationSamplingMap(void);
};
